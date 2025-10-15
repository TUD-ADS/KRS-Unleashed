
#ifndef MINIMAL_IMAGE_PUBLISHER_HPP_
#define MINIMAL_IMAGE_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>			   // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>				   // We include everything about OpenCV as we don't care much about compilation time at the moment.

using namespace std::chrono_literals;
namespace minimal_image_pub
{

	class MinimalImagePublisher : public rclcpp::Node
	{
	public:
		MinimalImagePublisher();
		MinimalImagePublisher(std::string image_name);

	private:
		cv::Mat GenerateRandomImage();
		cv::Mat LoadGrayScaleImage();

		void TimerCallback();

		rclcpp::TimerBase::SharedPtr timer_;
		sensor_msgs::msg::Image::SharedPtr msg_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
		size_t count_;
		std::string m_image_name;

	}; // class MinimalImagePublisher

} // namespace minimal_image_pub

#endif
