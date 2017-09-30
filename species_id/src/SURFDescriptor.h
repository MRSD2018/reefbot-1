// A SURF descriptor of an image. These are found in an image by using
// the SURFGenerator class.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: June 2010

#ifndef _SPECIES_ID_SURF_DESCRIPTOR_H__
#define _SPECIES_ID_SURF_DESCRIPTOR_H__

#include "opencv2/features2d/features2d.hpp"
#include "ImageDescriptor.h"

namespace species_id {

class SURFDescriptor : public ImageDescriptor<float> {
public:
  // Constructors
  SURFDescriptor(const cv::KeyPoint& keypoint)
    : ImageDescriptor<float>(keypoint.pt), keypoint_(keypoint) {}

  template <class InputIterator>
  SURFDescriptor(const cv::KeyPoint& keypoint, InputIterator begin, InputIterator end)
    : ImageDescriptor<float>(keypoint.pt, begin, end), keypoint_(keypoint) {}

  // Swaps the contents of the descriptor instead of copying. The new
  // entires in desc are undefined. If you want to copy, then use the
  // reference based constructor
  SURFDescriptor(const cv::KeyPoint& keypoint, std::vector<float>* desc)
    : ImageDescriptor<float>(keypoint.pt, desc), keypoint_(keypoint) {}

  // This will copy the values from the input descriptor. To swap, use
  // the pointer based constructor instead.
  SURFDescriptor(const cv::KeyPoint& keypoint, const std::vector<float>& desc)
    : ImageDescriptor<float>(keypoint.pt, desc), keypoint_(keypoint) {}

  virtual ~SURFDescriptor();

  const cv::KeyPoint& keypoint() { return keypoint_; }
private:
  SURFDescriptor();

  cv::KeyPoint keypoint_;
};

}

#endif
