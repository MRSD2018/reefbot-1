// Abstract class for a set of image descriptors like SIFT or SURF features.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: June 2010

#ifndef _SPECIES_ID_IMAGE_DESCRIPTOR_H__
#define _SPECIES_ID_IMAGE_DESCRIPTOR_H__

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/core/core.hpp"

namespace species_id {

template <typename T>
class ImageDescriptor {
public:
  static const float UNKNOWN_LOCATION = -1.0;
  typedef boost::shared_ptr<ImageDescriptor<T> > Ptr;
  typedef boost::shared_ptr<ImageDescriptor<T> const> ConstPtr;

  // Constructors
  ImageDescriptor()
    : point_(UNKNOWN_LOCATION, UNKNOWN_LOCATION),
      hasValidPoint_(false),
      descriptor_() {}
  ImageDescriptor(const cv::Point2f& pt)
    : point_(pt),
      hasValidPoint_(true),
      descriptor_() {}
  template <class InputIterator>
  ImageDescriptor(const cv::Point2f& pt, InputIterator begin, InputIterator end)
    : point_(pt),
      hasValidPoint_(true),
      descriptor_(begin, end) {}

  // Swaps the contents of the descriptor instead of copying. The new
  // entires in desc are undefined. If you want to copy, then use the
  // reference based constructor
  ImageDescriptor(const cv::Point2f& pt, std::vector<T>* desc)
    : point_(pt),
      hasValidPoint_(true),
      descriptor_() {
    descriptor_.swap(*desc);
  }
  
  // This will copy the values from the input descriptor. To swap, use
  // the pointer based constructor instead.
  ImageDescriptor(const cv::Point2f& pt, const std::vector<T>& desc)
    : point_(pt), hasValidPoint_(true), descriptor_(desc) {}

  // This will copy the values from the input descriptor. To swap, use
  // the pointer based constructor instead.
  ImageDescriptor(const cv::Point2f& pt, const T& val)
    : point_(pt), hasValidPoint_(true), descriptor_(1, val) {}

  virtual ~ImageDescriptor() {}

  virtual const T GetVal(int index) const { return descriptor_[index]; }
  virtual void SetVal(int index, const T& val) {descriptor_[index] = val;}

  const cv::Point2f& point() const { return point_;}
  bool hasValidPoint() const {
    return hasValidPoint_ && 
      point_.x != UNKNOWN_LOCATION &&
      point_.y != UNKNOWN_LOCATION; }
  
  virtual int DescriptorSize() const { return descriptor_.size(); }

  template <class InputIterator>
  void AppendVals(InputIterator begin, InputIterator end) {
    descriptor_.insert(descriptor_.end(), begin, end);
  }

protected:
  // Location of the descriptor
  cv::Point2f point_;
  bool hasValidPoint_;

  // The actual descriptor
  std::vector<T> descriptor_;

};

// Class the serializes an image descriptor into the format used by
// the INRIA Feature extrator by Krystian.Mikolajczyk@inrialpes.fr.
//
// The format is:
//
// vector_dimension (e.g. size of descriptor)
// nb_of_descriptors
// x y a b c desc_1 desc_2 ......desc_vector_dimension
//
// Note, a, b and c are always 0 (unused)
//
// To use this class, on, for example cout, do:
//
// ImageDescriptorINRIASerializer serializer;
// serializer.serialize(cout, imageDescriptors)
//
// Can also be chained, like:
//
// serializer.Write(cout, imageDescriptors) << 32;
//
// or:
// 
// cout << 32 << serializer.Write(cout, imageDescriptors);
template <typename T>
class ImageDescriptorINRIASerializer {
public:
  // Default constructor
  ImageDescriptorINRIASerializer() {}

  // Output a single ImageDescriptor. i.e. output:
  // x y a b c desc_1 desc_2 ......desc_vector_dimension
  std::ostream& Write(std::ostream& stream,
                      const ImageDescriptor<T>& descriptor) const;

  // Outputs the header. (i.e:
  // vector_dimension (e.g. size of descriptor)
  // nb_of_descriptors <newline>
  std::ostream& WriteHeader(
    std::ostream& stream,
    const std::vector<typename ImageDescriptor<T>::Ptr>& descriptors) const;

  std::ostream& WriteHeader(std::ostream& stream,
                            int descriptorSize,
                            int nDescriptors) const {
    return stream << descriptorSize << std::endl
                  << nDescriptors << std::endl;
  }

  // Output a set of descriptors. This includes writing the header.
  std::ostream& Write(
    std::ostream& stream,
    const std::vector<typename ImageDescriptor<T>::Ptr>& descriptors) const;
  
};

}

#endif
