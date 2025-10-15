#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include "apriltag_utils/msg/hulls_result.hpp"
#include "apriltag_utils/msg/quads_v2.hpp"
#include "tracetools_kernels/tracetools.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <chrono>
#include <cstdint>

#include "apriltag/TagDetector.h"
#include "apriltag_cpp/params.hpp"
#include "apriltag_cpp/message_serializer.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/extremal_polygon_2.h>

size_t get_hulls_msg_size(apriltag_utils::msg::HullsResult::ConstSharedPtr msg)
{
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<apriltag_utils::msg::HullsResult> msg_serialization;
    const void *ptr = reinterpret_cast<const void *>(msg.get());
    msg_serialization.serialize_message(ptr, &serialized_data_img);
    size_t msg_size = serialized_data_img.size();
    return msg_size;
}

static bool ccw(const at::Point &p1,
                const at::Point &p2,
                const at::Point &p3)
{

    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x) > 0;
}

namespace apriltag_cpp
{

    class ApriltagQuadsNode : public rclcpp::Node
    {
    public:
        ApriltagQuadsNode()
            : Node("apriltag_quads_node"), family("Tag36h11"), params(), detector(family, params)
        {
            subscription_ = this->create_subscription<apriltag_utils::msg::HullsResult>(
                "/apriltag/hulls", 2, std::bind(&ApriltagQuadsNode::topic_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<apriltag_utils::msg::QuadsV2>("/apriltag/quads", 2);
            debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/quads/debug", 10);
            opticalCenter = cv::Point2d(apriltag::center_factor * apriltag::image_width,
                                        apriltag::center_factor * apriltag::image_height);

            TRACEPOINT(kernel_register,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       "quads_kernel_cpu");
        }

    private:
        TagFamily family;
        TagDetectorParams params;
        TagDetector detector;
        cv::Point2d opticalCenter;
        rclcpp::Subscription<apriltag_utils::msg::HullsResult>::SharedPtr subscription_;
        rclcpp::Publisher<apriltag_utils::msg::QuadsV2>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;

        void topic_callback(const apriltag_utils::msg::HullsResult::SharedPtr msg) const
        {
            std::vector<std::vector<cv::Point2i>> hulls;
            int current_index = 0;

            std::stringstream ss;
            ss << "\n";

            for (size_t hulls_index = 0; hulls_index < msg->hulls_index.size(); hulls_index++) // hulls
            {
                std::vector<cv::Point2i> hull;
                for (int point_index = 0; point_index < msg->hulls_index[hulls_index]; point_index++)
                {
                    cv::Point2i point{msg->hulls_blob[current_index], msg->hulls_blob[current_index + 1]};
                    hull.push_back(point);
                    current_index += 2;
                }
                hulls.push_back(hull);
            }

            std::vector<at::real> hareas = msg->hareas;

            TRACEPOINT(kernel_start,
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       static_cast<const void *>(&(*msg)),
                       msg->header.stamp.nanosec,
                       msg->header.stamp.sec,
                       get_hulls_msg_size(msg));

            typedef CGAL::Simple_cartesian<int> K;
            typedef K::Point_2 Point;
            typedef CGAL::Polygon_2<K> Polygon_2;

            std::vector<float> observedPerimeters;
            std::vector<std::vector<cv::Point2f>> quad_points;

            for (size_t i = 0; i < hulls.size(); ++i)
            {
                ss << "hull: " << i << std::endl;

                Polygon_2 cpoly;
                for (size_t j = 0; j < hulls[i].size(); ++j)
                {
                    cpoly.push_back(Point(hulls[i][j].x, hulls[i][j].y));
                }

                Polygon_2 k_gon;
                CGAL::maximum_area_inscribed_k_gon_2(cpoly.vertices_begin(),
                                                     cpoly.vertices_end(), 4,
                                                     std::back_inserter(k_gon));

                at::Point p[4];

                bool ok = true;

                for (int j = 0; j < 4; ++j)
                {
                    const Point &ki = k_gon[4 - j - 1];
                    if (ki.x() <= 1 || ki.x() >= apriltag::image_width - 2 ||
                        ki.y() <= 1 || ki.y() >= apriltag::image_height - 2)
                    {
                        ok = false;
                        ss << "Rejected due to pixels required outside" << std::endl;
                        break;
                    }
                    p[j] = at::Point(ki.x() + 0.5, ki.y() + 0.5);
                }

                for (auto &point : p)
                {
                    ss << "(" << point.x << ", " << point.y << ")";
                }
                ss << std::endl;

                if (ok)
                {
                    at::real pa = area(p, 4);
                    at::real ha = hareas[i];
                    assert(ha >= pa);
                    if (pa / ha < at::real(0.7)) // 0.8
                    {
                        ok = false;
                        ss << "Rejected due to estimated hull structure not being similar enough to polygon" << std::endl;
                    }
                }

                if (ok && ccw(p[0], p[1], p[2]))
                {
                    std::swap(p[0], p[3]);
                    std::swap(p[1], p[2]);
                }

                at::real observedPerimeter = 0;
                at::real dmax = 0;
                at::real dmin = apriltag::image_width + apriltag::image_height;

                if (ok)
                {

                    for (int i = 0; i < 4; ++i)
                    {
                        int j = (i + 1) % 4;
                        at::Point diff = p[i] - p[j];
                        at::real dij = sqrt(diff.dot(diff));
                        dmax = std::max(dmax, dij);
                        if (dij < params.minimumTagSize)
                        {
                            ok = false;
                            ss << "Rejected due to tag being too small" << std::endl;
                            break;
                        }
                        observedPerimeter += dij;
                    }

                    for (int i = 0; i < 4; ++i)
                    {
                        for (int j = 0; j < i; ++j)
                        {
                            at::Point diff = p[i] - p[j];
                            at::real dij = sqrt(diff.dot(diff));
                            dmin = std::min(dmin, dij);
                        }
                    }
                }
                if (dmax / dmin >= 12) // 6
                {
                    ok = false;
                    ss << "Rejected due to ??" << std::endl;
                }

                if (ok)
                {
                    observedPerimeters.push_back(observedPerimeter);
                    std::vector<cv::Point2f> points{p[0], p[1], p[2], p[3]};
                    quad_points.push_back(points);
                    ss << "Accepted" << std::endl;
                }
            }
            RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());

            TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer

            apriltag_utils::msg::QuadsV2 outMsg;
            outMsg.header = msg->header;
            outMsg.optical_center[0] = opticalCenter.x;
            outMsg.optical_center[1] = opticalCenter.y;
            outMsg.observed_perimeters = observedPerimeters;

            std::vector<float> flattened_points;
            for (size_t i = 0; i < quad_points.size(); i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    flattened_points.push_back(quad_points[i][j].x);
                    flattened_points.push_back(quad_points[i][j].y);
                }
            }
            outMsg.quads_blob = flattened_points;
            publisher_->publish(outMsg);

            if (apriltag::debug_images)
            {
                const std::vector<cv::Scalar> &ccolors = getCColors();
                cv::Mat drawing = cv::Mat::zeros(cv::Size(apriltag::image_width, apriltag::image_height), CV_8UC3);
                for (size_t i = 0; i < quad_points.size(); i++)
                {
                    cv::Scalar color = ccolors[i % ccolors.size()];
                    for (int j = 0; j < 4; ++j)
                    {
                        cv::line(drawing, quad_points[i][j], quad_points[i][(j + 1) % 4], color, 1, cv::LINE_8);
                    }
                }
                cv_bridge::CvImage cv_img = cv_bridge::CvImage(msg->header, "rgb8", drawing);
                sensor_msgs::msg::Image outImg;
                cv_img.toImageMsg(outImg);
                debug_publisher_->publish(outImg);
            }
        }
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<apriltag_cpp::ApriltagQuadsNode>());
    rclcpp::shutdown();
    return 0;
}
