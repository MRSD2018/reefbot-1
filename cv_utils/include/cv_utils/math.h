#ifndef __CV_UTILS_MATH_H__
#define __CV_UTILS_MATH_H__
#include <opencv2/core/core.hpp>

namespace cv_utils {

// Get the min and max values of an image
double min(const cv::Mat& image);
double max(const cv::Mat& image);
double sum(const cv::Mat& image, int channel=0);

} // namespace

#endif // __CV_UTILS_MATH_H__
