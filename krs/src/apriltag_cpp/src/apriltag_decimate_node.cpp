#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
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
    class ApriltagDecimatorNode : public rclcpp::Node
    {
    public:
        ApriltagDecimatorNode()
            : Node("apriltag_decimate_node"), family("Tag36h11"), params(), detector(family, params)
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/image_raw", 2, std::bind(&ApriltagDecimatorNode::topic_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/decimated", 2);

            TRACEPOINT(kernel_register,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       "decimate_kernel_cpu");
        }

    private:
        TagFamily family;
        TagDetectorParams params;
        TagDetector detector; // not needed, just here for tracing
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

        void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            TRACEPOINT(kernel_start,
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       static_cast<const void *>(&(*msg)),
                       msg->header.stamp.nanosec,
                       msg->header.stamp.sec,
                       get_msg_size(msg));

            cv::Mat gray;
            cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

            if (apriltag::decimate_factor > 1)
            {
                cv::Mat tmp;
                double resize_factor = 1 / apriltag::decimate_factor; // 2 would be 0.5
                cv::resize(gray, tmp, cv::Size(0, 0), resize_factor, resize_factor);
                gray = tmp;
            }

            TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer

            cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
            cv_ptr->image = gray;
            sensor_msgs::msg::Image outImg;
            cv_ptr->toImageMsg(outImg);
            publisher_->publish(outImg);
        }
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<apriltag_cpp::ApriltagDecimatorNode>());
    rclcpp::shutdown();
    return 0;
}