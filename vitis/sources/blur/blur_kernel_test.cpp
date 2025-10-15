/*
 * Copyright 2022 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common/xf_headers.hpp"
#include "blur/blur_kernel_test.hpp"

using namespace std;

int main()
{

    cv::Mat in_img, out_img, ocv_ref, julia_ref;
    cv::Mat diff, diff_julia;

    string in_image_path = "blur_in.png";
    in_img = cv::imread(in_image_path, 0); // reading in the color image
    if (!in_img.data)
    {
        fprintf(stderr, "Failed to load the input image ... !!!\n ");
        return -1;
    }

    string result_image_path = "blur_result.png";
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

#if FILTER_WIDTH == 3
    float sigma = 0.5f;
#endif
#if FILTER_WIDTH == 7
    float sigma = 1.16666f;
#endif
#if FILTER_WIDTH == 5
    float sigma = 0.8333f;
#endif

    int height = in_img.rows;
    int width = in_img.cols;

    // OpenCV Gaussian filter function
    cv::GaussianBlur(in_img, ocv_ref, cv::Size(FILTER_WIDTH, FILTER_WIDTH), sigma, sigma, cv::BORDER_CONSTANT);
    imwrite("output_ocv.png", ocv_ref);

    // Call the top function
    blur::gaussian_filter_accel((ap_uint<blur::INPUT_PTR_WIDTH> *)in_img.data, (ap_uint<blur::OUTPUT_PTR_WIDTH> *)out_img.data, height,
                                width, sigma);

    // Write output image
    cv::imwrite("blur_out.png", out_img);

    // Compute absolute difference image
    cv::absdiff(ocv_ref, out_img, diff);

    imwrite("error.png", diff); // Save the difference image for debugging purpose

    float err_per;
    xf::cv::analyzeDiff(diff, blur::ERROR_THRESHOLD, err_per);

    if (err_per > 0.0f)
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
    xf::cv::analyzeDiff(diff_julia, blur::ERROR_THRESHOLD, err_per);

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
