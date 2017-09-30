// A SIFT descriptor of an image. These are found in an image by using
// the SIFTGenerator class.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: June 2010

#ifndef _SPECIES_ID_SIFT_DESCRIPTOR_H__
#define _SPECIES_ID_SIFT_DESCRIPTOR_H__

#include "opencv2/features2d/features2d.hpp"
#include "ImageDescriptor.h"

namespace species_id {

class SIFTDescriptor : public ImageDescriptor<float> {
public:
  // Constructors
  SIFTDescriptor(const cv::KeyPoint& keypoint)
    : ImageDescriptor<float>(keypoint.pt), keypoint_(keypoint) {}

  template <class InputIterator>
  SIFTDescriptor(const cv::KeyPoint& keypoint, InputIterator begin, InputIterator end)
    : ImageDescriptor<float>(keypoint.pt, begin, end), keypoint_(keypoint) {}

  // Swaps the contents of the descriptor instead of copying. The new
  // entires in desc are undefined. If you want to copy, then use the
  // reference based constructor
  SIFTDescriptor(const cv::KeyPoint& keypoint, std::vector<float>* desc)
    : ImageDescriptor<float>(keypoint.pt, desc), keypoint_(keypoint) {}

  // This will copy the values from the input descriptor. To swap, use
  // the pointer based constructor instead.
  SIFTDescriptor(const cv::KeyPoint& keypoint, const std::vector<float>& desc)
    : ImageDescriptor<float>(keypoint.pt, desc), keypoint_(keypoint) {}

  virtual ~SIFTDescriptor();

  const cv::KeyPoint& keypoint() { return keypoint_; }
private:
  SIFTDescriptor();

  cv::KeyPoint keypoint_;
};

}

#endif
