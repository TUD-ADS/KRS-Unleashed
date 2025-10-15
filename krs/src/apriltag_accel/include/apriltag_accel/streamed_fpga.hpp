
#ifndef STREAMED_FPGA_HPP_
#define STREAMED_FPGA_HPP_

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
    class StreamedFPGA : public rclcpp::Node
    {
    public:
        StreamedFPGA(const rclcpp::NodeOptions &options);

        ~StreamedFPGA()
        {
            // Free memory from OpenCL objects
            delete krnl_decimate_;
            delete krnl_blur_;
            delete krnl_threshold_;
            delete context_;
            delete queue_;
            delete tracer_;

        } // AcceleratedComponentFPGA destructor

    protected:
        cl::Kernel *krnl_decimate_;
        cl::Kernel *krnl_blur_;
        cl::Kernel *krnl_threshold_;
        cl::Context *context_;
        cl::CommandQueue *queue_;
        std::mutex connect_mutex_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_gray_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_threshold_;

        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;

        image_transport::Subscriber subscriber_;

        void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);
        size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg);

    private:
        cv_bridge::CvImagePtr cv_ptr;         // Stores input image from img_msg in  subscriber callback
        cv::Mat result_gray_hls;              // stores intermediary result after kernel execution on FPGA
        cv::Mat result_hls;                   // stores result after kernels execution on FPGA
                                              // use result pointer as ID
        cv_bridge::CvImage output_image;      // Create CV image from result_hls, required to publish image msg.
        cv_bridge::CvImage output_gray_image; // Create CV image from result_gray_hls, required to publish image msg.

        void InitKernel();
        void ExecuteKernel();

#if FILTER_WIDTH == 3
        float sigma = 0.5f;
#endif
#if FILTER_WIDTH == 7
        float sigma = 1.16666f;
#endif
#if FILTER_WIDTH == 5
        float sigma = 0.8333f;
#endif

    }; // class StreamedFPGA

} // namespace apriltag_accel

#endif // STREAMED_FPGA_HPP_
