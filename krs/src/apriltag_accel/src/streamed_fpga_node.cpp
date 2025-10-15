#include <memory>
#include "apriltag_accel/streamed_fpga.hpp"

using namespace std::chrono_literals; // NOLINT

int main(int argc, char *argv[])
{
    // ROS 2 abstractions
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<apriltag_accel::StreamedFPGA>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}