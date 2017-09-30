#include "cv_utils/DisplayImages.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

namespace cv_utils {

void DisplayNormalizedImage(const cv::Mat& image, const char* windowName) {
  Mat outImage = image;
  if (image.channels() == 1) {
    normalize(image, outImage, 0, 255, NORM_MINMAX,
              CV_MAKETYPE(CV_8U, image.channels()));
  }

  
  cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
  
  cvShowImage(windowName, &IplImage(outImage));
}

void DisplayLabImage(const cv::Mat& image, const char* windowName) {
  Mat outImage;
  Mat outImage32;
  image.convertTo(outImage32, CV_32FC3);
  cvtColor(outImage32, outImage, CV_Lab2BGR);
  
  cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
  
  cvShowImage(windowName, &IplImage(outImage));
}

void DisplayImageWithBoxes(const cv::Mat& image,
                           const std::vector<cv::Rect>& boxes,
                           const char* windowName) {
  Mat outImage = image.clone();

  for (vector<Rect>::const_iterator i = boxes.begin();
       i != boxes.end(); ++i) {
    rectangle(outImage, *i, Scalar(255, 0, 0));
  }
  
  cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
  
  cvShowImage(windowName, &IplImage(outImage));      
}

void ShowWindowsUntilKeyPress() {
  int key = -1;
  while (key < 0) {
    key = cvWaitKey(100);
  }
}

} // namespace 
