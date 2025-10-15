#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include "apriltag_utils/msg/contours_result.hpp"
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

namespace apriltag_cpp
{

    class ApriltagContoursComponent : public rclcpp::Node
    {
    public:
        ApriltagContoursComponent(const rclcpp::NodeOptions &options)
            : Node("apriltag_contours_component", options), family("Tag36h11"), params(), detector(family, params)
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/apriltag/thresholded", 2, std::bind(&ApriltagContoursComponent::topic_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<apriltag_utils::msg::ContoursResult>("/apriltag/contours", 2);
            debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/contours/debug", 10);

            TRACEPOINT(kernel_register,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       "contours_kernel_cpu");
        }

    private:
        TagFamily family;
        TagDetectorParams params;
        TagDetector detector;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Publisher<apriltag_utils::msg::ContoursResult>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
        cv::Mat thresh;

        void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

            TRACEPOINT(kernel_start,
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       static_cast<const void *>(&(*msg)),
                       msg->header.stamp.nanosec,
                       msg->header.stamp.sec,
                       get_msg_size(msg));
            std::vector<std::vector<cv::Point2i>> contours;

            std::vector<cv::Vec4i> hierarchy;

            cv::findContours(cv_ptr->image, contours, hierarchy,
                             cv::RETR_CCOMP,
                             cv::CHAIN_APPROX_SIMPLE);

            TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer

            apriltag_utils::msg::ContoursResult outMsg;
            outMsg.header = msg->header;
            std::vector<int> index;
            std::vector<int> flattened_points;
            for (size_t i = 0; i < contours.size(); i++)
            {
                index.push_back(contours[i].size()); // save size of inner array, which consists of points <x,y>

                for (int j = 0; j < contours[i].size(); j++)
                {
                    flattened_points.push_back(contours[i][j].x);
                    flattened_points.push_back(contours[i][j].y);
                }
            }
            std::vector<int> flattened_hierarchy;
            for (const auto &v : hierarchy)
            {
                flattened_hierarchy.push_back(v[0]);
                flattened_hierarchy.push_back(v[1]);
                flattened_hierarchy.push_back(v[2]);
                flattened_hierarchy.push_back(v[3]);
            }

            outMsg.contours_index = index;
            outMsg.contours_blob = flattened_points;
            outMsg.hierarchy_blob = flattened_hierarchy;

            publisher_->publish(outMsg);

            if (apriltag::debug_images)
            {
                const std::vector<cv::Scalar> &ccolors = getCColors();
                cv::Mat drawing = cv::Mat::zeros(cv_ptr->image.size(), CV_8UC3);
                at::Point delta(0.5, 0.5);
                for (size_t i = 0; i < contours.size(); i++)
                {
                    cv::Scalar color = ccolors[i % ccolors.size()];
                    cv::drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
                }
                cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
                cv_ptr->image = drawing;
                sensor_msgs::msg::Image outImg;
                cv_ptr->toImageMsg(outImg);
                debug_publisher_->publish(outImg);
            }
        }
    };
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_cpp::ApriltagContoursComponent)
