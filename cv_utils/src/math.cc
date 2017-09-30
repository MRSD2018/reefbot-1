#include "cv_utils/math.h"

using namespace cv;

namespace cv_utils {

double min(const Mat& image) {
  double minVal;
  minMaxLoc(image, &minVal, NULL, NULL, NULL);
  return minVal;
}

double max(const Mat& image) {
  double maxVal;
  minMaxLoc(image, NULL, &maxVal, NULL, NULL);
  return maxVal;
}

double sum(const Mat& image, int channel) {
  return cv::sum(image)[channel];
}

} // namespace
