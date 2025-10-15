#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include "apriltag_utils/msg/quads_v2.hpp"
#include "apriltag_utils/msg/apriltag_detections.hpp"
#include "tracetools_kernels/tracetools.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <fstream>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "apriltag/TagDetector.h"
#include "apriltag_cpp/params.hpp"
#include "apriltag_cpp/message_serializer.hpp"
#include "apriltag/GrayModel.h"

size_t get_quads_msg_size(apriltag_utils::msg::QuadsV2::ConstSharedPtr msg)
{
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<apriltag_utils::msg::QuadsV2> msg_serialization;
    const void *ptr = reinterpret_cast<const void *>(msg.get());
    msg_serialization.serialize_message(ptr, &serialized_data_img);
    size_t msg_size = serialized_data_img.size();
    return msg_size;
}

bool detectionsOverlapTooMuch1(const TagDetection &a, const TagDetection &b)
{
    // Compute a sort of "radius" of the two targets. We'll do
    // this by computing the average length of the edges of the
    // quads (in pixels).
    at::real radius = 0.0625 * (pdist(a.p[0], a.p[1]) +
                                pdist(a.p[1], a.p[2]) +
                                pdist(a.p[2], a.p[3]) +
                                pdist(a.p[3], a.p[0]) +
                                pdist(b.p[0], b.p[1]) +
                                pdist(b.p[1], b.p[2]) +
                                pdist(b.p[2], b.p[3]) +
                                pdist(b.p[3], b.p[0]));

    // distance (in pixels) between two tag centers.
    at::real d = pdist(a.cxy, b.cxy);

    // reject pairs where the distance between centroids is
    // smaller than the "radius" of one of the tags.
    return (d < radius);
}

bool decodeQuad(const cv::Mat_<unsigned char> &image,
                const cv::Mat &orig_image,
                const Quad &quad, size_t i, const TagFamily &tagFamily, std::stringstream &ss,
                TagDetectionArray &detections)
{

    int width = apriltag::image_width, height = apriltag::image_height;

    GrayModel blackModel, whiteModel;
    ;

    // sample points around the black and white border in
    // order to calibrate our gray threshold. This code is
    // simpler if we loop over the whole rectangle and discard
    // the points we don't want.
    int dd = 2 * tagFamily.blackBorder + tagFamily.d;

    std::vector<cv::Point> bpoints, wpoints;
    at::real gscl = 1.0 / 255;

    for (int iy = -1; iy <= dd; iy++)
    {
        for (int ix = -1; ix <= dd; ix++)
        {

            at::real y = (iy + .5) / dd;
            at::real x = (ix + .5) / dd;

            at::Point pxy = interpolate(quad.p, at::Point(x, y));

            at::real v = bicubicInterpolate(image, pxy) * gscl;

            if (pxy.x < 0 || pxy.x >= width || pxy.y < 0 || pxy.y >= height)
            {
                continue;
            }

            if ((iy == -1 || iy == dd) || (ix == -1 || ix == dd))
            {
                // part of the outer white border.
                whiteModel.addObservation(x, y, v);
                if (apriltag::debug_images)
                {
                    wpoints.push_back(pxy);
                }
            }
            else if ((iy == 0 || iy == (dd - 1)) || (ix == 0 || ix == (dd - 1)))
            {
                // part of the outer black border.
                blackModel.addObservation(x, y, v);
                if (apriltag::debug_images)
                {
                    bpoints.push_back(pxy);
                }
            }
        }
    }

    bool bad = false;
    TagFamily::code_t tagCode = 0;

    // Try reading off the bits.
    // XXX: todo: multiple samples within each cell and vote?
    for (at::uint iy = tagFamily.d - 1; iy < tagFamily.d; iy--)
    {

        if (apriltag::debug_images)
        {
            ss << "  ";
        }

        for (at::uint ix = 0; ix < tagFamily.d; ix++)
        {

            at::real y = (tagFamily.blackBorder + iy + .5) / dd;
            at::real x = (tagFamily.blackBorder + ix + .5) / dd;

            at::Point pxy = interpolate(quad.p, at::Point(x, y));

            at::real v = bicubicInterpolate(image, pxy) * gscl;

            if (pxy.x < 0 || pxy.x >= width || pxy.y < 0 || pxy.y >= height)
            {
                if (apriltag::debug_images)
                {
                    ss << "quad " << i << " was bad!\n";
                }
                bad = true;
                continue;
            }

            at::real threshold = (blackModel.interpolate(x, y) +
                                  whiteModel.interpolate(x, y)) *
                                 .5;

            tagCode = tagCode << TagFamily::code_t(1);

            if (v > threshold)
            {
                tagCode |= TagFamily::code_t(1);
                if (apriltag::debug_images)
                {
                    wpoints.push_back(pxy);
                }
            }
            else
            {
                if (apriltag::debug_images)
                {
                    bpoints.push_back(pxy);
                }
            }

            if (apriltag::debug_images)
            {
                ss << ((v > threshold) ? "##" : "  ");
            }
        }

        if (apriltag::debug_images)
        {
            ss << "\n";
        }
    }

    if (apriltag::debug_images)
    {
        ss << "\n";
    }

    if (!bad)
    {

        TagDetection d;
        tagFamily.decode(d, tagCode);

        if (apriltag::debug_images)
        {

            ss << "for quad " << i << " got tagCode " << tagCode << "\n";
            ss << "closest tag is " << d.code << " and good is " << d.good << "\n";
        }

        // rotate points in detection according to decoded
        // orientation. Thus the order of the points in the
        // detection object can be used to determine the
        // orientation of the target.

        for (int i = 0; i < 4; i++)
        {
            d.p[(4 + i - d.rotation) % 4] = quad.p[i];
        }

        // compute the homography (and rotate it appropriately)
        d.homography = quad.H;
        d.hxy = quad.opticalCenter;

        if (true)
        {
            at::real c = cos(d.rotation * M_PI / 2.0);
            at::real s = sin(d.rotation * M_PI / 2.0);
            at::real R[9] = {
                c, -s, 0,
                s, c, 0,
                0, 0, 1};
            at::Mat Rmat(3, 3, R);
            d.homography = d.homography * Rmat;
        }

        if (d.good)
        {
            d.cxy = interpolate(quad.p, at::Point(0.5, 0.5));
            d.observedPerimeter = quad.observedPerimeter;
            detections.push_back(d);
            return true;
        }
    }
    return false;
}

namespace apriltag_cpp
{

    class ApriltagTagDecodeComponent : public rclcpp::Node
    {
    public:
        ApriltagTagDecodeComponent(const rclcpp::NodeOptions &options)
            : Node("apriltag_tag_decode_component", options), family("Tag36h11"), params(), detector(family, params)
        {
            subscriber_original_image.subscribe(this, "/image_raw");
            subscriber_gray_image.subscribe(this, "/apriltag/decimated");
            subscriber_quads.subscribe(this, "/apriltag/quads");

            sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(2), subscriber_original_image, subscriber_gray_image, subscriber_quads));
            sync_->registerCallback(std::bind(&ApriltagTagDecodeComponent::combined_callback,
                                              this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

            publisher_ = this->create_publisher<apriltag_utils::msg::ApriltagDetections>("/apriltag/tag_decode", 10);
            debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/apriltag/tag_decode/debug", 10);
            opticalCenter = cv::Point2d(apriltag::center_factor * apriltag::image_width,
                                        apriltag::center_factor * apriltag::image_height);

            TRACEPOINT(kernel_register,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       "tag_decode_kernel_cpu");
        }

    private:
        TagFamily family;
        TagDetectorParams params;
        TagDetector detector;
        cv::Point2d opticalCenter;

        message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_original_image;
        message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_gray_image;
        message_filters::Subscriber<apriltag_utils::msg::QuadsV2> subscriber_quads;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, apriltag_utils::msg::QuadsV2> MySyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

        rclcpp::Publisher<apriltag_utils::msg::ApriltagDetections>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;

        void combined_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr &msgImgOrig,
            const sensor_msgs::msg::Image::ConstSharedPtr &msgImgGray,
            const apriltag_utils::msg::QuadsV2::ConstSharedPtr &msgQuads) const
        {

            std::vector<std::vector<cv::Point2f>> quad_points;

            for (size_t quads_index = 0; quads_index < msgQuads->quads_blob.size(); quads_index += 8) // 4-quad group of x,y points
            {

                std::vector<cv::Point2f> quad_group = {
                    {msgQuads->quads_blob[quads_index], msgQuads->quads_blob[quads_index + 1]},
                    {msgQuads->quads_blob[quads_index + 2], msgQuads->quads_blob[quads_index + 3]},
                    {msgQuads->quads_blob[quads_index + 4], msgQuads->quads_blob[quads_index + 5]},
                    {msgQuads->quads_blob[quads_index + 6], msgQuads->quads_blob[quads_index + 7]},
                };

                quad_points.push_back(quad_group);
            }

            std::vector<float> observedPerimeters = msgQuads->observed_perimeters;
            cv::Point2d opticalCenter(msgQuads->optical_center[0], msgQuads->optical_center[1]);

            TagDetectionArray detections;
            cv_bridge::CvImagePtr cv_ptr_orig;
            cv_ptr_orig = cv_bridge::toCvCopy(msgImgOrig, "bgr8");
            cv_bridge::CvImagePtr cv_ptr_gray;
            cv_ptr_gray = cv_bridge::toCvCopy(msgImgGray, "mono8");

            TRACEPOINT(kernel_start,
                       static_cast<const void *>(&detector), // use detector as kernel pointer
                       static_cast<const void *>(&(*msgQuads)),
                       msgQuads->header.stamp.nanosec,
                       msgQuads->header.stamp.sec,
                       get_quads_msg_size(msgQuads));

            ////////////////////////////////////////////////////////////////
            // Step eight. Decode the quads. For each quad, we first
            // estimate a threshold color to decided between 0 and
            // 1. Then, we read off the bits and see if they make sense.

            for (size_t i = 0; i < quad_points.size(); ++i)
            {
                at::Point pose_array[4] = {quad_points[i][0],
                                           quad_points[i][1],
                                           quad_points[i][2],
                                           quad_points[i][3]};
                Quad quad = {pose_array,
                             opticalCenter,
                             observedPerimeters[i]};

                std::stringstream ss;
                ss << "\n";
                bool good = decodeQuad(cv_ptr_gray->image, cv_ptr_orig->image, quad, i, family, ss, detections);
                RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());
            }

            ////////////////////////////////////////////////////////////////
            // Step nine. Some quads may be detected more than once, due
            // to partial occlusion and our aggressive attempts to recover
            // from broken lines. When two quads (with the same id)
            // overlap, we will keep the one with the lowest error, and if
            // the error is the same, the one with the greatest observed
            // perimeter.

            TagDetectionArray goodDetections;

            // NOTE: allow multiple (non-overlapping) detections of the same target.
            for (size_t i = 0; i < detections.size(); ++i)
            {

                const TagDetection &d = detections[i];

                bool newFeature = true;

                for (size_t odidx = 0; odidx < goodDetections.size(); odidx++)
                {

                    const TagDetection &od = goodDetections[odidx];

                    if (d.id != od.id || !detectionsOverlapTooMuch1(d, od))
                    {
                        continue;
                    }

                    // there's a conflict. we must pick one to keep.
                    newFeature = false;

                    // this detection is worse than the previous one... just don't use it.
                    if (d.hammingDistance > od.hammingDistance)
                    {
                        continue;
                    }

                    // otherwise, keep the new one if it either has
                    // *lower* error, or has greater perimeter
                    if (d.hammingDistance < od.hammingDistance ||
                        d.observedPerimeter > od.observedPerimeter)
                    {
                        goodDetections[odidx] = d;
                    }
                }

                if (newFeature)
                {
                    goodDetections.push_back(d);
                }
            }

            goodDetections.swap(detections);

            TRACEPOINT(kernel_end, static_cast<const void *>(&detector)); // use detector as kernel pointer

            apriltag_utils::msg::ApriltagDetections outMsg;
            outMsg.header = msgQuads->header;
            std::vector<apriltag_utils::msg::ApriltagDetection> detections_container;
            for (auto &detection : detections)
            {
                apriltag_utils::msg::ApriltagDetection detect_msg;
                detect_msg.family_index = 1; // hardcoded for 36h11
                detect_msg.id = detection.id;
                detect_msg.hamming = detection.hammingDistance;
                detect_msg.decision_margin = detection.observedPerimeter;

                detect_msg.h[0] = detection.homography(0, 0);
                detect_msg.h[1] = detection.homography(0, 1);
                detect_msg.h[2] = detection.homography(0, 2);
                detect_msg.h[3] = detection.homography(1, 0);
                detect_msg.h[4] = detection.homography(1, 1);
                detect_msg.h[5] = detection.homography(1, 2);
                detect_msg.h[6] = detection.homography(2, 0);
                detect_msg.h[7] = detection.homography(2, 1);
                detect_msg.h[8] = detection.homography(2, 2);

                detect_msg.c[0] = detection.cxy.x;
                detect_msg.c[1] = detection.cxy.y;

                detect_msg.p[0] = detection.p[0].x;
                detect_msg.p[1] = detection.p[0].y;
                detect_msg.p[2] = detection.p[1].x;
                detect_msg.p[3] = detection.p[1].y;
                detect_msg.p[4] = detection.p[2].x;
                detect_msg.p[5] = detection.p[2].y;
                detect_msg.p[6] = detection.p[3].x;
                detect_msg.p[7] = detection.p[3].y;

                detections_container.push_back(detect_msg);
            }
            outMsg.detections = detections_container;
            publisher_->publish(outMsg);

            if (apriltag::debug_images)
            {
                cv::Mat drawing = cv_ptr_orig->image;

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
                cv_bridge::CvImage cv_img = cv_bridge::CvImage(msgImgOrig->header, "rgb8", drawing);
                sensor_msgs::msg::Image outImg;
                cv_img.toImageMsg(outImg);
                debug_publisher_->publish(outImg);
            }
        }
    };

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_cpp::ApriltagTagDecodeComponent)