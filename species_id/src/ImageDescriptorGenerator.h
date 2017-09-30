// Abstract class for a class that can create a set of
// ImageDescriptor's from an image. The descriptors inherently have a
// location in the image associated with them.
//
// Author: Mark Desnoyer
// Date: June 2010

#ifndef _SPECIES_ID_IMAGE_DESCRIPTOR_GENERATOR_H_
#define _SPECIES_ID_IMAGE_DESCRIPTOR_GENERATOR_H_

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "base/BasicTypes.h"
#include "reefbot_msgs/ImageRegion.h"
#include "sensor_msgs/Image.h"
#include "ImageDescriptor.h"
#include "RandomDetector.h"

namespace species_id {

template <typename T>
class ImageDescriptorGenerator {
public:
  typedef std::vector<reefbot_msgs::ImageRegion> MaskCollection;
  typedef typename ImageDescriptor<T>::Ptr DescriptorPtr;
  typedef std::vector<DescriptorPtr> DescriptorCollection;

  ImageDescriptorGenerator(const cv::FeatureDetector* keyGenerator)
    : keyGenerator_(keyGenerator), needsMono_(true) {}

  virtual ~ImageDescriptorGenerator() {}

  // Extracts a set of descriptors from an image for each mask region
  // that is defined.
  //
  // Inputs:
  // image - The image to extract the descriptors from
  // masks - Set of ImageRegions that define the masks to look for
  //         descriptors. If no masks are in the collection, uses
  //         the whole image.
  // 
  // Outputs:
  // descriptors - Set of descriptor collections, one for each mask
  //               that specifies the descriptors present in that mask.
  virtual void ExtractUsingMasks(
    const cv::Mat& image,
    const MaskCollection& masks,
    std::vector<boost::shared_ptr<DescriptorCollection> >* descriptors) const;                        

  // Instead of using a collection of masks for a given image, we can
  // call each mask independently. InitializeForWholeImage must be
  // called first
  //
  // Inputs:
  // image - The image to extract the descriptors from
  // mask - Mask to get the descriptor of
  //
  // Outputs:
  // The collection of descriptors found in that mask.
  virtual void ExtractFromSingleMask(
    const cv::Mat& image,
    const reefbot_msgs::ImageRegion& mask,
    DescriptorCollection* collection) const;

  // Initializes the entire image so that calls to ExtractFromSingleMask
  // can execute efficiently.
  // 
  // Inputs:
  // image - Image to initialize for.
  // mask - Optional boolean mask that specifies locations in the
  // image where it is valid to have a descriptors. N.B. If this is
  // used, it is the user's responsibility to make sure that only the
  // regions specified will be requested. No error will be reported if
  // you use it wrong.
  virtual void InitializeForWholeImage(
    const cv::Mat& image,
    const cv::Mat_<uint8>& mask=cv::Mat_<uint8>());

protected:
  const cv::FeatureDetector* keyGenerator_;

  bool needsMono_; // Does the image need to be greyscale?

  // Function defined by the subclasses that find the descriptors
  // throughout the entire image.
  //
  // Inputs:
  // image - The image to extract the descriptors from
  // keypoints - Keypoints in the image to get the descriptor for.
  // 
  // Outputs:
  // descriptors - Collections of ImageDescriptors that were found in the
  //               image.
  virtual void ExtractFromWholeImage(
    const cv::Mat& image,
    const std::vector<cv::KeyPoint>& keypoints,
    DescriptorCollection* descriptors) const = 0;

  ImageDescriptorGenerator()
    : keyGenerator_(NULL), needsMono_(false) {}

private:
  // Returns true if the mask constains the point
  bool MaskContainsPoint(const reefbot_msgs::ImageRegion& mask,
                         const cv::Point2f& point) const;

  // Returns true if the mask contains the descriptor
  bool MaskContainsDescriptor(const reefbot_msgs::ImageRegion& mask,
                              const ImageDescriptor<T>& descriptor) const;

  // Takes the points defined in masks and labels them in the combined mask
  void AddToCombinedMasks(const reefbot_msgs::ImageRegion& mask,
                          cv::Mat_<uchar>& combinedMask) const;

  mutable sensor_msgs::Image::Ptr maskCache_;

  boost::scoped_ptr<DescriptorCollection> wholeImageDescriptors_;
  const void* curImageData_; // The data for the current image contained in wholeImageDescriptors_. Used to make sure we call InitializeForWholeImage first.

  // Evil constructors
  DISALLOW_EVIL_CONSTRUCTORS(ImageDescriptorGenerator);
};

}

#endif //_SPECIES_ID_IMAGE_DESCRIPTOR_GENERATOR_H_
