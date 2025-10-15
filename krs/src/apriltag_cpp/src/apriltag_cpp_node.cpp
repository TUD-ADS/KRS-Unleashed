#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/serialization.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include "tracetools_kernels/tracetools.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "apriltag/TagDetector.h"
#include "apriltag_cpp/message_serializer.hpp"

class ApriltagCppNode : public rclcpp::Node
{
public:
    ApriltagCppNode()
        : Node("apriltag_cpp_node"), family("Tag36h11"), params(), detector(family, params)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            imgTopic, 10, std::bind(&ApriltagCppNode::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/debug_detection", 10);

        detector.debug = show_debug_info;
        detector.debugWindowName = "";
        detector.params.newQuadAlgorithm = true;
        detector.params.segSigma = 0; // 1.999f; // check with apriltag::blur_sigma;
        if (params.segDecimate && be_verbose)
        {
            std::cout << "Will decimate for segmentation!\n";
        }

        TRACEPOINT(kernel_register,
                   static_cast<const void *>(this),
                   static_cast<const void *>(&detector), // use detector as kernel pointer
                   "apriltag_kernel_cpu");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    bool show_debug_info = false;
    bool show_timing = false;
    bool show_results = false;
    bool be_verbose = false;
    bool no_images = false;
    bool generate_output_files = false;
    TagFamily family;
    TagDetectorParams params;
    TagDetector detector;
    std::string imgTopic = "/image_raw";

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        RCLCPP_DEBUG(this->get_logger(), "Callback, got data!");

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); // todo

        TRACEPOINT(kernel_start,
                   static_cast<const void *>(&detector), // use detector as kernel pointer
                   static_cast<const void *>(&(*msg)),
                   msg->header.stamp.nanosec,
                   msg->header.stamp.sec,
                   get_msg_size(msg));

        cv::Mat gray;
        cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        TagDetectionArray detections;

        // while (std::max(gray.rows, gray.cols) > 800)
        // {
        //     cv::Mat tmp;
        //     cv::resize(gray, tmp, cv::Size(0, 0), 0.5, 0.5);
        //     gray = tmp;
        // }
        cv::Point2d opticalCenter(0.5 * gray.rows, 0.5 * gray.cols);
        RCLCPP_DEBUG(this->get_logger(), "Start detector");
        detector.process(gray, opticalCenter, detections);

        TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer
        RCLCPP_DEBUG(this->get_logger(), "Detector done");
        // cv::Mat img = family.superimposeDetections(cv_ptr->image, detections);

        cv::Mat drawing = cv_ptr->image;

        for (auto &detection : detections)
        {
            cv::Scalar color = CV_RGB(204, 0, 0);
            if (detection.good)
            {
                color = CV_RGB(0, 204, 0);
            }
            for (int p_index = 0; p_index < 4; p_index++)
            {
                cv::line(drawing, detection.p[p_index], detection.p[(p_index + 1) % 4], color, 1, cv::LINE_8);
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Debug Image generation done");
        cv_ptr->image = drawing;
        sensor_msgs::msg::Image outImg;
        cv_ptr->toImageMsg(outImg);
        publisher_->publish(outImg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagCppNode>());
    rclcpp::shutdown();
    return 0;
}
