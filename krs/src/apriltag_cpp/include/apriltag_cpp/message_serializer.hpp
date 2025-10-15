#pragma once

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/serialization.hpp>
#include "sensor_msgs/msg/image.hpp"

size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg)
{
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    const void *image_ptr = reinterpret_cast<const void *>(image_msg.get());
    image_serialization.serialize_message(image_ptr, &serialized_data_img);
    size_t image_msg_size = serialized_data_img.size();
    return image_msg_size;
}