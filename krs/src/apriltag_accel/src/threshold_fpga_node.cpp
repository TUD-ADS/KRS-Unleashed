// Copyright (c) 2021, Xilinx®.
// All rights reserved
//
// Inspired by the Vector-Add example.
// See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis
//

#include <memory>
#include "apriltag_accel/threshold_fpga.hpp"

using namespace std::chrono_literals; // NOLINT

int main(int argc, char *argv[])
{
    // ROS 2 abstractions
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<apriltag_accel::ThresholdFPGA>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
