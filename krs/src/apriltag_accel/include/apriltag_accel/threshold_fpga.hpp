
#ifndef THRESHOLD_FPGA_HPP_
#define THRESHOLD_FPGA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <vitis_common/common/ros_opencl_120.hpp>

#include "xf_blur_config.h"

namespace apriltag_accel
{
  class ThresholdFPGA : public rclcpp::Node
  {
  public:
    ThresholdFPGA(const rclcpp::NodeOptions &options);

    ~ThresholdFPGA()
    {
      // Free memory from OpenCL objects
      delete krnl_;
      delete context_;
      delete queue_;
      delete tracer_;

    } // AcceleratedComponentFPGA destructor

  protected:
    cl::Kernel *krnl_;
    cl::Context *context_;
    cl::CommandQueue *queue_;
    std::mutex connect_mutex_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;

    image_transport::Subscriber subscriber_;

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);
    size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg);

  private:
    cv_bridge::CvImagePtr cv_ptr;    // Stores input image from img_msg in  subscriber callback
    cv::Mat result_hls;              // stores result after kernel execution on FPGA
                                     // use result pointer as ID
    cv_bridge::CvImage output_image; // Create CV image from result_hls, required to publish image msg.

    void InitKernel();
    void ExecuteKernel();

  }; // class ThresholdFPGA

} // namespace apriltag_accel

#endif // THRESHOLD_FPGA_HPP_
