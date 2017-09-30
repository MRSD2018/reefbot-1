// OpenCV object that acts as an integral histogram for an image.
// An integral histogram is a (h+1) x (w+1) x nbins matrix where each bin plan is
// an integral image of the counts in that bin.
//
// Only handles single channel images right now.
//
// Copyright 2012 Mark Desnoyer (mdesnoyer@gmail.com

#ifndef __CV_INTEGRAL_HISTOGRAM_H__
#define __CV_INTEGRAL_HISTOGRAM_H__

#include <opencv2/core/core.hpp>
#include <vector>
#include <cstdlib>

namespace cv_utils {

template <typename T>
class IntegralHistogram : public cv::Mat_<T> {
public:
  virtual ~IntegralHistogram();

  // Type of interpolation to perform
  typedef enum {
    NO_INTERP,
    LINEAR_INTERP
  } InterpType;

  // Create an integral histogram using uniform bin sizes. Caller must
  // take ownership of the created object.
  //
  // Inputs:
  // image - Image to calculate the histogram for. It is h x w 
  // nbins - The number of bins in the histogram
  // range - The minimum and maximum values for the histogram.
  //         If NULL, then they are calculated as the min and max values
  //         in the image.
  // weights - Optional weight for each pixel in the image. If empty, it is 1.
  // binInterp - Optional interpolation between bins. This will cause votes 
  //             to accumulate in the two bins around the point.
  template <typename ImageT>
  static IntegralHistogram<T>* Calculate(
    const cv::Mat_<ImageT>& image,
    int nbins,
    const std::pair<ImageT, ImageT>* range = NULL,
    const cv::Mat_<ImageT>& weights=cv::Mat_<ImageT>(),
    InterpType binInterp=NO_INTERP);

  // Get the histogram at a given pixel as a 1-D array
  const cv::Mat_<T> GetArrayHist(int row, int col) const;
  const cv::Mat_<T> GetArrayHist(const cv::Point& pt) const {
    return GetArrayHist(pt.y, pt.x);
  }

  // Get the histogram in a rectangle
  cv::Mat_<T> GetHistInRegion(const cv::Rect& rect) const;
  void GetHistInRegion(const cv::Rect& rect, std::vector<T>* out) const {
    return GetHistInRegion(rect.x, rect.y, rect.width, rect.height, out);
  }
  void GetHistInRegion(int x, int y, int w, int h, std::vector<T>* out,
                       const float normFactor=1.0) const;
  void GetHistInRegion(int x, int y, int w, int h, T* out,
                       const float normFactor=1.0) const;

  // Getters
  int nbins() const { return nbins_; }
  int randId() const { return randId_; }

private:
  // Copies of important features so we can avoid a vptr lookup
  int stepY_;
  int stepX_;
  int nbins_;
  uchar* histStart_;

  // A random id generated when the object is built so that we can
  // identify it uniquely. Memory location doesn't work because
  // sometimes the compiler will put the new object at the same spot
  // on the stack, when going around a loop.
  int randId_;
  

  // Evil constructor
  IntegralHistogram();

  // Internal constructor to get the sizes right
  IntegralHistogram(const int* sizes) {
    cv::Mat_<T>::create(3, sizes);
    stepY_ = this->step[0];
    stepX_ = this->step[1];
    nbins_ = sizes[2];
    histStart_ = this->data;
    randId_ = std::rand();
  }

};

} // namespace

#endif // __CV_INTEGRAL_HISTOGRAM_H__
