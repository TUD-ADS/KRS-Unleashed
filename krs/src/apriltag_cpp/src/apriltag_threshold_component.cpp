#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tracetools_kernels/tracetools.h"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

#include "apriltag/TagDetector.h"
#include "apriltag_cpp/message_serializer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_cpp
{

  class ApriltagThresholdComponent : public rclcpp::Node
  {
  public:
    ApriltagThresholdComponent(const rclcpp::NodeOptions &options)
        : Node("apriltag_threshold_component", options), family("Tag36h11"), params(), detector(family, params)
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/apriltag/decimated", 2, std::bind(&ApriltagThresholdComponent::topic_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/thresholded", 2);

      TRACEPOINT(kernel_register,
                 static_cast<const void *>(this),
                 static_cast<const void *>(&detector), // use detector as kernel pointer
                 "threshold_kernel_cpu");
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
      // begin
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

      // prepare
      TRACEPOINT(kernel_start,
                 static_cast<const void *>(&detector), // use detector as kernel pointer
                 static_cast<const void *>(&(*msg)),
                 msg->header.stamp.nanosec,
                 msg->header.stamp.sec,
                 get_msg_size(msg));

      cv::Mat thresh;

      cv::adaptiveThreshold(cv_ptr->image, thresh, 255,
                            cv::ADAPTIVE_THRESH_MEAN_C,
                            cv::THRESH_BINARY_INV,
                            params.adaptiveThresholdRadius,
                            params.adaptiveThresholdValue);

      // calculate
      TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer

      cv_ptr->image = thresh;
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
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_cpp::ApriltagThresholdComponent)
