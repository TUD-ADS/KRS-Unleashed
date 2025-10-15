#include "common/xf_headers.hpp"
#include "threshold/threshold_kernel_test.hpp"

using namespace std;

int main()
{

    cv::Mat in_img, out_img, ocv_ref, julia_ref;
    cv::Mat diff, diff_julia;

    string in_image_path = "threshold_in.png";
    in_img = cv::imread(in_image_path, 0); // reading in the color image
    if (!in_img.data)
    {
        fprintf(stderr, "Failed to load the input image ... !!!\n ");
        return -1;
    }

    string result_image_path = "threshold_result.png";
    julia_ref = cv::imread(result_image_path, 0); // reading in the color image
    if (!julia_ref.data)
    {
        fprintf(stderr, "Failed to load the result image ... !!!\n ");
        return -1;
    }

    out_img.create(in_img.rows, in_img.cols, CV_8UC1);    // create memory for output image
    diff.create(in_img.rows, in_img.cols, CV_8UC1);       // create memory for OCV-ref image
    diff_julia.create(in_img.rows, in_img.cols, CV_8UC1); // create memory for OCV-ref image
    ocv_ref.create(in_img.rows, in_img.cols, CV_8UC1);    // create memory for OCV-ref image

    int height = in_img.rows;
    int width = in_img.cols;

    // OpenCV Adaptive Threshold filter function
    cv::adaptiveThreshold(in_img, ocv_ref, threshold::ADAPTIVE_MAX_VALUE,
                          cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY_INV,
                          threshold::adaptive_threshold_radius,
                          threshold::adaptive_threshold_delta);

    imwrite("output_ocv.png", ocv_ref);

    // Call the top function
    threshold::adaptive_threshold((ap_uint<threshold::INPUT_PTR_WIDTH> *)in_img.data,
                                  (ap_uint<threshold::OUTPUT_PTR_WIDTH> *)out_img.data);

    // Write output image
    cv::imwrite("threshold_out.png", out_img);

    // Compute absolute difference image
    cv::absdiff(ocv_ref, out_img, diff);

    imwrite("error.png", diff); // Save the difference image for debugging purpose

    float err_per;
    xf::cv::analyzeDiff(diff, threshold::ERROR_THRESHOLD, err_per);

    if (err_per > 1.0f)
    {
        fprintf(stderr, "ERROR: OpenCV Test Failed.\n ");
        return 1;
    }
    else
    {
        std::cout << "OpenCV Test Passed " << std::endl;
    }

    cv::absdiff(julia_ref, out_img, diff_julia);

    imwrite("error_julia.png", diff_julia); // Save the difference image for debugging purpose

    err_per = 0.0f;
    xf::cv::analyzeDiff(diff_julia, threshold::ERROR_THRESHOLD, err_per);

    // skip Julia test for now
    // if (err_per > 20.0f)
    // {
    //     fprintf(stderr, "ERROR: Julia Refererence Test Failed.\n ");
    //     return 1;
    // }
    // else
    // {
    //     std::cout << "Julia Refererence Test Passed " << std::endl;
    // }

    return 0;
}
