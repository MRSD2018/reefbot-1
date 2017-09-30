// Some hash functions for opencv data types to be used with unordered_map
// and unordered_set
//
// Author: Mark Desnoyer
// Date: Oct 2012

#ifndef _CV_UTILS_HASH_FUNCS_H__
#define _CV_UTILS_HASH_FUNCS_H__
#include <opencv2/core/core.hpp>

namespace cv {

template <typename T>
std::size_t hash_value(const cv::Point_<T>& point) {
  std::size_t seed = 54981;
  hash_combine(seed, point.x);
  hash_combine(seed, point.y);
  return seed;
}

template <typename T>
std::size_t hash_value(const cv::Size_<T>& size) {
  std::size_t seed = 54981;
  hash_combine(seed, size.width);
  hash_combine(seed, size.height);
  return seed;
}

template <typename T>
std::size_t hash_value(const cv::Rect_<T>& rect) {
  std::size_t seed = 54981;
  hash_combine(seed, rect.x);
  hash_combine(seed, rect.y);
  hash_combine(seed, rect.width);
  hash_combine(seed, rect.height);
  return seed;
}

} // namespace

#endif // _CV_UTILS_HASH_FUNCS_H__
