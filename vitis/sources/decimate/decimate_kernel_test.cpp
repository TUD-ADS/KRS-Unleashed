#include "common/xf_headers.hpp"
#include "decimate/decimate_kernel_test.hpp"

using namespace std;

int main(int argc, char **argv)
{
    cv::Mat in_img, out_img, ocv_ref, julia_ref;
    cv::Mat diff, diff_julia;

    string in_image_path = "decimate_in.png";
    in_img = cv::imread(in_image_path, 0); // reading in the color image
    if (!in_img.data)
    {
        fprintf(stderr, "Failed to load the input image ... !!!\n ");
        return -1;
    }

    string result_image_path = "decimate_result.png";
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

    cvtColor(in_img, in_img, cv::COLOR_BGR2RGB); // necessary as imread always converts to BGR

    // OpenCV Gray Conversion function
    cvtColor(in_img, ocv_ref, cv::COLOR_RGB2GRAY);
    imwrite("output_ocv.png", ocv_ref);

    // Call the top function
    decimate::cvtcolor_rgb2gray((ap_uint<decimate::INPUT_PTR_WIDTH> *)in_img.data, (ap_uint<decimate::OUTPUT_PTR_WIDTH> *)out_img.data);

    // Write output image
    cv::imwrite("decimate_out.png", out_img);

    // Compute absolute difference image
    cv::absdiff(ocv_ref, out_img, diff);

    imwrite("error.png", diff); // Save the difference image for debugging purpose

    float err_per;
    xf::cv::analyzeDiff(diff, decimate::ERROR_THRESHOLD, err_per);

    if (err_per > 3.0f)
    {
        fprintf(stderr, "ERROR: OpenCV Test Failed.\n");
        return 1;
    }
    else
    {
        std::cout << "OpenCV Test Passed " << std::endl;
    }

    cv::absdiff(julia_ref, out_img, diff_julia);

    imwrite("error_julia.png", diff_julia); // Save the difference image for debugging purpose

    err_per = 0.0f;
    xf::cv::analyzeDiff(diff_julia, decimate::ERROR_THRESHOLD, err_per);

    if (err_per > 20.0f)
    {
        fprintf(stderr, "ERROR: Julia Refererence Test Failed.\n ");
        return 1;
    }
    else
    {
        std::cout << "Julia Refererence Test Passed " << std::endl;
    }

    return 0;
}