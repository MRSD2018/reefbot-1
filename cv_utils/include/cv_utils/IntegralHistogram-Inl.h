#include "cv_utils/IntegralHistogram.h"
#include <ros/ros.h>

namespace cv_utils {


template<typename T>
const T& SafeIntegralValue(const cv::Mat_<T>& mat, int row, int col,
                           int bin) {
  static const T ZERO = 0.0;
  if (row < 0 || col < 0) {
    return ZERO;
  }
  return mat(row, col, bin);
}

template<typename T>
IntegralHistogram<T>::~IntegralHistogram() {
  this->release();
}

template<typename T>
template<typename ImageT>
IntegralHistogram<T>* IntegralHistogram<T>::Calculate(
    const cv::Mat_<ImageT>& image, int nbins,
    const std::pair<ImageT, ImageT>* range,
    const cv::Mat_<ImageT>& weights,
    InterpType binInterp) {
  // Intialize the histogram
  const int sizes[3] = {image.rows+1, image.cols+1, nbins};
  IntegralHistogram<T>* retval = new IntegralHistogram<T>(sizes);
  for (int row=0; row <= image.rows; ++row) {
    T* pixPtr = reinterpret_cast<T*>(retval->ptr(row, 0, 0));
    for (int bin = 0; bin < nbins; ++bin) {
      (*pixPtr++) = 0;
    }
  }
  T* pixPtr = reinterpret_cast<T*>(retval->ptr(0, 0, 0));
  for (int col = 0; col <= image.cols; ++col) {
    for (int bin = 0; bin < nbins; ++bin) {
      (*pixPtr++) = 0;
    }
  }

  // Only single channel arrays allowed for now
  ROS_ASSERT(image.channels() == 1);
  
  // Weights must be the same size as the image
  ROS_ASSERT(weights.empty() ||
             (weights.rows == image.rows && weights.cols == image.cols));

  // Figure out the range of the bins
  double minVal;
  double maxVal;
  if (range == NULL) {
    minMaxLoc(image, &minVal, &maxVal, NULL, NULL);
  } else {
    minVal = range->first;
    maxVal = range->second;
  }
  double binSize = (maxVal - minVal+1) / nbins;

  // Calculate the histogram
  for(int row = 0; row < image.rows; ++row) {
    const ImageT* imagePtr = image[row];
    const ImageT* weightPtr = weights.empty() ? NULL : weights[row];
    for (int col = 0; col < image.cols; ++col) {
      const T& curVal = imagePtr[col];
      const T& curW = weights.empty() ? 1.0 : weightPtr[col];

      // Figure out what bin this pixel should go in
      double curBin;
      if (curVal < minVal || curVal > maxVal) {
        // Won't be put in a bin
        curBin = -10;
      } else {
        curBin = (curVal == maxVal) ? nbins - 1 :
          static_cast<double>(curVal - minVal) / binSize;
      }

      T* destPtr = reinterpret_cast<T*>(retval->ptr(row+1,col+1,0));
      T* lastRowPtr = reinterpret_cast<T*>(retval->ptr(row,col+1,0));
      T* lastColPtr = reinterpret_cast<T*>(retval->ptr(row+1,col,0));
      T* lastCornerPtr = reinterpret_cast<T*>(retval->ptr(row,col,0));

      // Fill out the histogram for this pixel
      for (int bin = 0; bin < nbins; ++bin) {
        T toAdd;
        if (binInterp == NO_INTERP) {
          toAdd = (bin == static_cast<int>(curBin)) ? 1 : 0;
        } else if (binInterp == LINEAR_INTERP) {
          // If the bin center is one of the two bracketing the ideal
          // location, add a linearily weighted amount of the weight
          // to this bin.
          toAdd = 1 - fabs(curBin - bin);
        } else {
          ROS_FATAL("Need to have an interpolations scheme");
          return NULL;
        }

        T newVal = (*lastRowPtr++) + (*lastColPtr++) - (*lastCornerPtr++);

        if (toAdd > 0) {
          newVal += toAdd * curW;
        }

        (*destPtr++) = newVal;
      }
    }
  }
  return retval;
}

template <typename T>
const cv::Mat_<T> IntegralHistogram<T>::GetArrayHist(int row, int col)  const{
  if (row < 0 || col < 0) {
    return cv::Mat_<double>::zeros(this->size.p[2], 1);
  } else {
    int trueRow = std::min(row, this->size.p[0]-1);
    int trueCol = std::min(col, this->size.p[1]-1);
    // Const cast is allowed because we're returning a const matrix
    // and the opencv library doesn't have a constructor that handles
    // the const.
    return cv::Mat_<T>(this->size.p[2], 1,
                       const_cast<T*>(&(*this)(trueRow, trueCol, 0)),
                       this->elemSize());
  }
}

template <typename T>
cv::Mat_<T> IntegralHistogram<T>::GetHistInRegion(const cv::Rect& rect)  const{
  cv::Mat_<T> retval(this->size.p[2], 1);

  GetHistInRegion(rect.x, rect.y, rect.width, rect.height, &(retval(0)));

  return retval;
}

template <typename T>
void IntegralHistogram<T>::GetHistInRegion(
  int x, int y, int w, int h, std::vector<T>* out,
  const float normFactor)  const {
#ifdef DEBUG
  ROS_ASSERT(out && out->size() == this->size.p[2]);
#endif
  GetHistInRegion(x, y, w, h, &((*out)[0]), normFactor);
}

template <typename T>
void IntegralHistogram<T>::GetHistInRegion(
  int x, int y, int w, int h, T* out, const float normFactor)  const {
  const uchar* basePtr = histStart_ + stepY_*y + stepX_*x;
  const T* tl = (T*)(basePtr);
  const int xStep = stepX_*w;
  const T* tr = (T*)(basePtr + xStep);
  const T* bl = (T*)(basePtr + stepY_*h);
  const T* br = (T*)((uchar*)(bl) + xStep);

  const int histSize = this->size.p[2];
  int i = 0;
  for (; i <= histSize - 4; i += 4) {
    out[i] = tl[i] + br[i] - tr[i] - bl[i];
    out[i+1] = tl[i+1] + br[i+1] - tr[i+1] - bl[i+1];
    out[i+2] = tl[i+2] + br[i+2] - tr[i+2] - bl[i+2];
    out[i+3] = tl[i+3] + br[i+3] - tr[i+3] - bl[i+3];
  }
  for (; i < histSize; ++i) {
    out[i] = tl[i] + br[i] - tr[i] - bl[i];
  }
}

} // namespace
