#include <cstdio>
#include "minimal_image_pub/minimal_image_pub.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace minimal_image_pub
{

  MinimalImagePublisher::MinimalImagePublisher() : Node("random_image_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    timer_ = this->create_wall_timer(30ms, std::bind(&MinimalImagePublisher::TimerCallback, this));
    m_image_name = "";
  }

  MinimalImagePublisher::MinimalImagePublisher(std::string image_name) : Node("opencv_image_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/decimated", 10);
    timer_ = this->create_wall_timer(30ms, std::bind(&MinimalImagePublisher::TimerCallback, this));
    m_image_name = image_name;
  }

  cv::Mat MinimalImagePublisher::GenerateRandomImage()
  {
    // Create a new 640x480 image
    cv::Mat my_image(cv::Size(640, 480), CV_8UC3);

    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    return my_image;
  }

  cv::Mat MinimalImagePublisher::LoadGrayScaleImage()
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("minimal_image_pub");
    std::string imagePath = package_share_directory + "/../../../build/minimal_image_pub/" + m_image_name + ".png"; // TODO check how to copy to share dir

    cv::Mat grayscaleImage = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (grayscaleImage.data == nullptr)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Image object empty, couldn't read file properly, please check at %s !", imagePath.c_str());
    }
    return grayscaleImage;
  }

  void MinimalImagePublisher::TimerCallback()
  {

    if (m_image_name.empty())
    {
      cv::Mat my_image = this->GenerateRandomImage();

      // Write message to be sent. Member function toImageMsg() converts a CvImage
      // into a ROS image message
      msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
                 .toImageMsg();
    }
    else
    {
      cv::Mat my_image = LoadGrayScaleImage();
      // Write message to be sent. Member function toImageMsg() converts a CvImage
      // into a ROS image message
      msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", my_image)
                 .toImageMsg();
    }

    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_DEBUG(this->get_logger(), "Image published, count = %ld ", count_);
    count_++;
  }
} // namespace minimal_image_pub

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<minimal_image_pub::MinimalImagePublisher>("input"));
  rclcpp::shutdown();
  return 0;
}
