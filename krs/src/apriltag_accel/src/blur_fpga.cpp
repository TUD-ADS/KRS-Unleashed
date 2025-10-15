/*
	Modification Copyright (c) 2023, Acceleration Robotics®
	Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
	Based on:
	  ____  ____
	 /   /\/   /
	/___/  \  /   Copyright (c) 2023, Xilinx®.
	\   \   \/    Author: Jasvinder Khurana <jasvinder.khurana@amd.com>
	 \   \
	 /   /        Licensed under the Apache License, Version 2.0 (the "License");
	/___/   /\    you may not use this file except in compliance with the License.
	\   \  /  \   You may obtain a copy of the License at
	 \___\/\___\            http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.

	Inspired by resize.cpp authored by Kentaro Wada, Joshua Whitley
*/

#include <mutex>
#include <chrono>

#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>

#include <common/xf_headers.hpp>
#include <vitis_common/common/utilities.hpp>

#include "apriltag_accel/blur_fpga.hpp"

#include "tracetools_kernels/tracetools.h"

// Forward declaration of utility functions included at the end of this file
std::vector<cl::Device> get_xilinx_devices();
char *read_binary_file(const std::string &xclbin_file_name, unsigned &nb);

#define XCLBIN_NAME "/lib/firmware/xilinx/smarobix_apriltag/smarobix_apriltag.xclbin"
#define KERNEL_NAME "gaussian_filter_accel"

namespace apriltag_accel
{

	void BlurFPGA::InitKernel()
	{
		cl_int err;
		unsigned fileBufSize;

		// Get the device:
		std::vector<cl::Device> devices = get_xilinx_devices();
		cl::Device device = devices[0];

		// Context, command queue and device name:
		OCL_CHECK(err, context_ = new cl::Context(device, NULL, NULL, NULL, &err));
		OCL_CHECK(err, queue_ = new cl::CommandQueue(*context_, device, CL_QUEUE_PROFILING_ENABLE, &err));
		OCL_CHECK(err, std::string device_name = device.getInfo<CL_DEVICE_NAME>(&err));

		RCLCPP_INFO(this->get_logger(), "INFO: Device found - %s", device_name.c_str());

		char *fileBuf = read_binary_file(XCLBIN_NAME, fileBufSize);

		cl::Program::Binaries bins{{fileBuf, fileBufSize}};
		devices.resize(1);
		OCL_CHECK(err, cl::Program program(*context_, devices, bins, NULL, &err));

		// Create a kernel:
		OCL_CHECK(err, krnl_ = new cl::Kernel(program, KERNEL_NAME, &err));
	}

	void BlurFPGA::ExecuteKernel()
	{
		int rows = HEIGHT;
		int cols = WIDTH;

		// OpenCL section:
		cl_int err;

		// OpenCL section:
		size_t image_in_size_bytes = rows * cols * sizeof(unsigned char);
		size_t image_out_size_bytes = rows * cols * sizeof(unsigned char); // todo check if CV_OUT_TYPE better here

		result_hls.create(rows, cols, CV_OUT_TYPE);

		// Allocate the buffers:
		OCL_CHECK(err, cl::Buffer buffer_inImage(*context_, CL_MEM_READ_ONLY, image_in_size_bytes, NULL, &err));
		OCL_CHECK(err, cl::Buffer buffer_outImage(*context_, CL_MEM_WRITE_ONLY, image_out_size_bytes, NULL, &err));

		// Set kernel arguments:
		OCL_CHECK(err, err = krnl_->setArg(0, buffer_inImage));
		OCL_CHECK(err, err = krnl_->setArg(1, buffer_outImage));
		OCL_CHECK(err, err = krnl_->setArg(2, rows));
		OCL_CHECK(err, err = krnl_->setArg(3, cols));
		OCL_CHECK(err, err = krnl_->setArg(4, sigma));

		RCLCPP_DEBUG(this->get_logger(), "OpenCL Kernel configured");

		// Initialize the buffers:
		cl::Event write_event, task_event, read_event;

		OCL_CHECK(err, queue_->enqueueWriteBuffer(buffer_inImage,	   // buffer on the FPGA
												  CL_TRUE,			   // blocking call
												  0,				   // buffer offset in bytes
												  image_in_size_bytes, // Size in bytes
												  cv_ptr->image.data,  // Pointer to the data to copy
												  nullptr, &write_event));

		RCLCPP_DEBUG(this->get_logger(), "OpenCL Job prepared");

		// Execute the kernel:
		OCL_CHECK(err, err = queue_->enqueueTask(*krnl_, nullptr, &task_event));

		RCLCPP_DEBUG(this->get_logger(), "Kernel executed");
		// Copy Result from Device Global Memory to Host Local Memory
		queue_->enqueueReadBuffer(buffer_outImage, // This buffers data will be read
								  CL_TRUE,		   // blocking call
								  0,			   // offset
								  image_out_size_bytes,
								  result_hls.data, // Data will be stored here
								  nullptr, &read_event);

		RCLCPP_DEBUG(this->get_logger(), "Extracted results from OpenCL Output Buffer");
		// Clean up:
		queue_->finish();

		output_image.header = cv_ptr->header;
		output_image.encoding = "mono8";

		output_image.image = cv::Mat{rows, cols, CV_8U, result_hls.data};
	}

	BlurFPGA::BlurFPGA(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
		: rclcpp::Node("apriltag_blur_fpga_node", options)
	{
		// Create image pub
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/blurred", 10);
		image_transport::TransportHints hints(this, "raw");
		subscriber_ = image_transport::create_subscription(
			this, "/apriltag/decimated",
			std::bind(&BlurFPGA::imageCb, this, std::placeholders::_1),
			hints.getTransport());

		InitKernel();
		TRACEPOINT(kernel_register,
				   static_cast<const void *>(this),
				   static_cast<const void *>(&result_hls), // use result as kernel pointer
				   "blur_kernel_fpga");
	}

	void BlurFPGA::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
	{

		// Get subscribed image
		//-------------------------------------------------------------------------------------------------------

		// Converting ROS image messages to OpenCV images, for diggestion
		// with the Vitis Vision Library
		// see http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
		RCLCPP_DEBUG(this->get_logger(), "Inside imageCb function");

		try
		{
			cv_ptr = cv_bridge::toCvCopy(image_msg);
		}
		catch (cv_bridge::Exception &e)
		{
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
			return;
		}

		// Process Image and run the accelerated Kernel
		//-------------------------------------------------------------------------------------------------------

		// this->get_parameter("profile", profile_);  // Update profile_
		RCLCPP_DEBUG(this->get_logger(), "Image converted to Mat object");
		TRACEPOINT(kernel_start,
				   static_cast<const void *>(&result_hls), // use result as kernel pointer
				   static_cast<const void *>(&(*image_msg)),
				   image_msg->header.stamp.nanosec,
				   image_msg->header.stamp.sec,
				   get_msg_size(image_msg));

		ExecuteKernel();
		TRACEPOINT(kernel_end, static_cast<const void *>(&result_hls)); // use result as kernel pointer
		sensor_msgs::msg::Image::SharedPtr msg_ = output_image.toImageMsg();

		// Publish processed image
		//-------------------------------------------------------------------------------------------------------
		publisher_->publish(*msg_.get());
	}

	size_t BlurFPGA::get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg)
	{
		rclcpp::SerializedMessage serialized_data_img;
		rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
		const void *image_ptr = reinterpret_cast<const void *>(image_msg.get());
		image_serialization.serialize_message(image_ptr, &serialized_data_img);
		size_t image_msg_size = serialized_data_img.size();
		return image_msg_size;
	}

} // namespace apriltag_accel

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_accel::BlurFPGA)
