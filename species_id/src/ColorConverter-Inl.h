#ifndef _SPECIES_ID_COLOR_CONVERTER_INL_H__
#define _SPECIES_ID_COLOR_CONVERTER_INL_H__

#include "ColorConverter.h"
#include <cmath>
#include <limits>

namespace species_id {

template<typename T>
OpponentColorConverter<T>::~OpponentColorConverter() {}

template<typename T>
void OpponentColorConverter<T>::ConvertImage(const cv::Mat& src,
                                             cv::Mat* dest) {
  assert(dest != NULL);
  if (!numeric_limits<T>::is_integer) {
    assert(src.type() == CV_32FC3 || src.type() == CV_64FC3);
  } else if (numeric_limits<T>::is_integer) {
    assert(src.type() == CV_32SC3 || 
           src.type() == CV_16UC3 || src.type() == CV_8UC3);
  }

  if (&src != dest) {
    dest->create(src.size(), src.type());
  }

  int cols = src.cols;
  int rows = src.rows;
  if (src.isContinuous() && dest->isContinuous()) {
    cols *= rows;
    rows = 1;
  }
  cols *= 3;

  const double SQRT_2 = sqrt((double)2);
  const double SQRT_6 = sqrt((double)6);
  const double SQRT_3 = sqrt((double)3);
  const double BIAS = numeric_limits<T>::is_integer ? numeric_limits<T>::max() / 2 : 0.5;

  for (int i = 0; i < rows; i++) {
    const T* srcPtr = src.ptr<T>(i);
    T* destPtr = dest->ptr<T>(i);

    for (int j = 0; j < cols; j += 3) {
      const double r = srcPtr[j+2];
      const double g = srcPtr[j+1];
      const double b = srcPtr[j];
      destPtr[j] = cv::saturate_cast<T>((r - g) / SQRT_2 + BIAS);
      destPtr[j+1] = cv::saturate_cast<T>((r + g - 2*b) / SQRT_6 + BIAS);
      destPtr[j+2] = cv::saturate_cast<T>((r + g + b) / SQRT_3);
    }
  }
}

template<typename T>
CInvariantColorConverter<T>::~CInvariantColorConverter() {}

template<typename T>
void CInvariantColorConverter<T>::ConvertImage(const cv::Mat& src,
                                               cv::Mat* dest) {
  assert(dest != NULL);
  assert(src.channels() == 3);
  if (!numeric_limits<T>::is_integer) {
    assert(src.type() == CV_32FC3 || src.type() == CV_64FC3);
  } else if (numeric_limits<T>::is_integer) {
    assert(src.type() == CV_32SC3 ||
           src.type() == CV_16UC3 || src.type() == CV_8UC3);
  }

  if (&src != dest) {
    dest->create(src.size(), src.type());
  }

  int cols = src.cols;
  int rows = src.rows;
  if (src.isContinuous() && dest->isContinuous()) {
    cols *= rows;
    rows = 1;
  }
  cols *= 3;

  const double SQRT_2 = sqrt((double)2);
  const double SQRT_6 = sqrt((double)6);
  const double SQRT_3 = sqrt((double)3);
  const double BIAS = numeric_limits<T>::is_integer ? numeric_limits<T>::max() / 2 : 0.5;
  const double SCALE = numeric_limits<T>::is_integer ? BIAS : 1.0;

  for (int i = 0; i < rows; i++) {
    const T* srcPtr = src.ptr<T>(i);
    T* destPtr = dest->ptr<T>(i);

    for (int j = 0; j < cols; j += 3) {
      const double r = srcPtr[j+2];
      const double g = srcPtr[j+1];
      const double b = srcPtr[j];
      const double o1 = (r - g) / SQRT_2;
      const double o2 = (r + g - 2*b) / SQRT_6;
      const double o3 = (r + g + b) / SQRT_3;
      destPtr[j] = cv::saturate_cast<T>(SCALE * o1 / o3 + BIAS);
      destPtr[j+1] = cv::saturate_cast<T>(SCALE * o2 / o3 + BIAS);
      destPtr[j+2] = cv::saturate_cast<T>(o3);
    }
  }
}

} // namespace

#endif // _SPECIES_ID_COLOR_CONVERTER_INL_H__
