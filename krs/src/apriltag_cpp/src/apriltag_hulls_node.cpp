#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include "apriltag_utils/msg/contours_result.hpp"
#include "apriltag_utils/msg/hulls_result.hpp"
#include "tracetools_kernels/tracetools.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <chrono>
#include <cstdint>

#include "apriltag/TagDetector.h"
#include "apriltag_cpp/message_serializer.hpp"
#include "apriltag_cpp/params.hpp"

size_t get_contours_msg_size(apriltag_utils::msg::ContoursResult::ConstSharedPtr msg)
{
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<apriltag_utils::msg::ContoursResult> msg_serialization;
    const void *ptr = reinterpret_cast<const void *>(msg.get());
    msg_serialization.serialize_message(ptr, &serialized_data_img);
    size_t msg_size = serialized_data_img.size();
    return msg_size;
}

namespace apriltag_cpp
{

    class ApriltagHullsNode : public rclcpp::Node
    {
    public:
        ApriltagHullsNode()
            : Node("apriltag_hulls_node"), family("Tag36h11"), params(), detector(family, params)
        {
            subscription_ = this->create_subscription<apriltag_utils::msg::ContoursResult>(
                "/apriltag/contours", 2, std::bind(&ApriltagHullsNode::topic_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<apriltag_utils::msg::HullsResult>("/apriltag/hulls", 2);
            debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/hulls/debug", 10);

            TRACEPOINT(kernel_register,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       "hulls_kernel_cpu");
        }

    private:
        TagFamily family;
        TagDetectorParams params;
        TagDetector detector;
        rclcpp::Subscription<apriltag_utils::msg::ContoursResult>::SharedPtr subscription_;
        rclcpp::Publisher<apriltag_utils::msg::HullsResult>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;

        void topic_callback(const apriltag_utils::msg::ContoursResult::SharedPtr msg) const
        {
            std::vector<std::vector<cv::Point2i>> contours;
            int current_index = 0;

            for (size_t contour_index = 0; contour_index < msg->contours_index.size(); contour_index++) // contours
            {
                std::vector<cv::Point2i> contour;
                for (int point_index = 0; point_index < msg->contours_index[contour_index]; point_index++)
                {
                    cv::Point2i point{msg->contours_blob[current_index], msg->contours_blob[current_index + 1]};
                    contour.push_back(point);
                    current_index += 2;
                }
                contours.push_back(contour);
            }

            std::vector<cv::Vec4i> hierarchy;
            for (int index = 0; index < msg->hierarchy_blob.size(); index += 4)
            {
                hierarchy.push_back(
                    {msg->hierarchy_blob[index],
                     msg->hierarchy_blob[index + 1],
                     msg->hierarchy_blob[index + 2],
                     msg->hierarchy_blob[index + 3]});
            }

            std::vector<std::vector<cv::Point2i>> hulls;
            std::vector<at::real> hareas;

            TRACEPOINT(kernel_start,
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       static_cast<const void *>(&(*msg)),
                       msg->header.stamp.nanosec,
                       msg->header.stamp.sec,
                       get_contours_msg_size(msg));

            int sl = family.d + 2 * family.blackBorder;
            int ta = sl * sl;

            for (size_t i = 0; i < contours.size(); ++i)
            {

                if (hierarchy[i][3] < 0 && contours[i].size() >= 4)
                {
                    std::vector<cv::Point2i> hull;
                    cv::convexHull(contours[i], hull);

                    at::real ca = area(&(contours[i][0]), contours[i].size());
                    at::real ha = area(&(hull[0]), hull.size());

                    assert(ha >= ca);

                    if (ca / ha > at::real(0.8) && ha >= ta)
                    {
                        hulls.push_back(hull);
                        hareas.push_back(ha);
                    }
                }
            }

            TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer

            apriltag_utils::msg::HullsResult outMsg;
            outMsg.header = msg->header;
            std::vector<int> index;
            std::vector<int> flattened_points;
            for (size_t i = 0; i < hulls.size(); i++)
            {
                index.push_back(hulls[i].size()); // save size of inner array, which consists of points <x,y>

                for (int j = 0; j < hulls[i].size(); j++)
                {
                    flattened_points.push_back(hulls[i][j].x);
                    flattened_points.push_back(hulls[i][j].y);
                }
            }

            outMsg.hulls_index = index;
            outMsg.hulls_blob = flattened_points;
            outMsg.hareas = hareas;

            publisher_->publish(outMsg);

            if (apriltag::debug_images)
            {
                const std::vector<cv::Scalar> &ccolors = getCColors();
                cv::Mat drawing = cv::Mat::zeros(cv::Size(apriltag::image_width, apriltag::image_height), CV_8UC3);
                for (size_t i = 0; i < hulls.size(); i++)
                {
                    cv::Scalar color = ccolors[i % ccolors.size()];
                    cv::drawContours(drawing, hulls, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
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
    rclcpp::spin(std::make_shared<apriltag_cpp::ApriltagHullsNode>());
    rclcpp::shutdown();
    return 0;
}
