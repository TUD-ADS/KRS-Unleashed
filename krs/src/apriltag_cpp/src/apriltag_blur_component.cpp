#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include "tracetools_kernels/tracetools.h"

#include "apriltag/TagDetector.h"
#include "apriltag_cpp/message_serializer.hpp"
#include "apriltag_cpp/params.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_cpp
{

    class ApriltagBlurComponent : public rclcpp::Node
    {
    public:
        ApriltagBlurComponent(const rclcpp::NodeOptions &options)
            : Node("apriltag_blur_component", options), family("Tag36h11"), params(), detector(family, params)
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/apriltag/decimated", 2, std::bind(&ApriltagBlurComponent::topic_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/blurred", 2);

            TRACEPOINT(kernel_register,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       "blur_kernel_cpu");
        }

    private:
        TagFamily family;
        TagDetectorParams params;
        TagDetector detector; // not needed, just here for tracing

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

        void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            RCLCPP_DEBUG(this->get_logger(), "Callback, got data!");
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
            cv::Mat blurred;

            TRACEPOINT(kernel_start,
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       static_cast<const void *>(&(*msg)),
                       msg->header.stamp.nanosec,
                       msg->header.stamp.sec,
                       get_msg_size(msg));

            if (apriltag::do_blur)
            {
                cv::GaussianBlur(cv_ptr->image, blurred, cv::Size(0, 0), apriltag::blur_sigma);
                cv_ptr->image = blurred;
            }

            TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer

            sensor_msgs::msg::Image outImg;
            cv_ptr->toImageMsg(outImg);
            publisher_->publish(outImg);
        }
    };
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_cpp::ApriltagBlurComponent)