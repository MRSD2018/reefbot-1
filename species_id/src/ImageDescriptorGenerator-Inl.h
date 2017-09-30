#ifndef _SPECIES_ID_IMAGE_DESCRIPTOR_GENERATOR_INL_H_
#define _SPECIES_ID_IMAGE_DESCRIPTOR_GENERATOR_INL_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <boost/unordered_set.hpp>
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "visual_utility/cvutils.h"

#include "ImageDescriptorGenerator.h"

using boost::shared_ptr;
using std::vector;
using namespace cv;
using reefbot_msgs::ImageRegion;
using boost::unordered_set;

namespace species_id {

template <typename T>
void ImageDescriptorGenerator<T>::ExtractUsingMasks(
  const Mat& image,
  const MaskCollection& masks,
  vector<shared_ptr<DescriptorCollection> >* descriptors) const {
  ROS_ASSERT(descriptors != NULL);
  ROS_ASSERT(keyGenerator_ != NULL);

  Mat monoImage;
  cvtColor(image, monoImage, CV_BGR2GRAY);

  const Mat* workingImage = &image;
  if (needsMono_) {
    workingImage = &monoImage;
  }

  ROS_DEBUG_STREAM("Extracting descriptors from an image of height: "
                   << workingImage->rows
                   << " width: "
                   << workingImage->cols
                   << " and type: "
                   << workingImage->type());

  ROS_DEBUG("Combining the Masks");
  Mat_<uchar> mask;
  if (!masks.empty()) {
    // TODO(mdesnoyer): Removing this because it doesn't speed things
    // up. The underlying hessian calculation is still done for the
    // entire image even with a mask.

    //mask = Mat_<uchar>::zeros(monoImage.rows, monoImage.cols);
    //for (MaskCollection::const_iterator maskI = masks.begin();
    //     maskI != masks.end();
    //     ++maskI) {
    //  maskCache_.reset();
    //
    //  AddToCombinedMasks(*maskI, mask);
    //}
  }

  ROS_DEBUG("Extracting Keypoints");
  // First get the keypoints
  vector<KeyPoint> keypoints;
  keyGenerator_->detect(monoImage, keypoints, mask);
  ROS_INFO_STREAM("Found " << keypoints.size() << " keypoints.");

  // Go through the keypoints and only keep those that are in a mask
  vector<KeyPoint> validKeypoints;
  unordered_set<const KeyPoint*> keypointSet;
  if (masks.empty()) {
    validKeypoints = keypoints;
  } else {
    for(MaskCollection::const_iterator maskI = masks.begin();
          maskI != masks.end();
          ++maskI) {
      maskCache_.reset();
      for(vector<KeyPoint>::const_iterator keyI = keypoints.begin();
          keyI != keypoints.end(); ++keyI) {
        if (MaskContainsPoint(*maskI, keyI->pt)) {
          pair<unordered_set<const KeyPoint*>::iterator, bool> result =
            keypointSet.insert(&(*keyI));
          if (result.second) {
            // It's a keypoint that wasn't added before, so keep track of it.
            validKeypoints.push_back(*keyI);
          }
        }
      }
    }
  }

  // Now get all the descriptors throughout the image
  ROS_INFO_STREAM("Extracting Descriptors from "
                  << validKeypoints.size() << " keypoints");
  shared_ptr<DescriptorCollection> allDescriptors(new DescriptorCollection());
  if (validKeypoints.size() > 0) {
    ExtractFromWholeImage(*workingImage, validKeypoints,
                          allDescriptors.get());
  } else {
    ROS_WARN("Did not find any valid keypoints for this image");
  }

  // Now, go through each mask and collect the descriptors that lie in
  // the mask.
  ROS_DEBUG("Filtering the descriptors");
  if (masks.empty()) {
    descriptors->push_back(allDescriptors);
  } else {
    for (MaskCollection::const_iterator maskI = masks.begin();
         maskI != masks.end();
         ++maskI) {
      maskCache_.reset();

      // Create the collection of descriptors for this mask
      shared_ptr<DescriptorCollection> curDescriptors(
        new DescriptorCollection());
      descriptors->push_back(curDescriptors);
      
      for (typename DescriptorCollection::const_iterator i =
             allDescriptors->begin();
           i != allDescriptors->end(); ++i) {
        if (MaskContainsDescriptor(*maskI, **i)) {
          curDescriptors->push_back(*i);
        }
      }
    }
  }
}

template <typename T>
void ImageDescriptorGenerator<T>::InitializeForWholeImage(
  const cv::Mat& image, const cv::Mat_<uint8>& mask) {
  Mat monoImage;
  cvtColor(image, monoImage, CV_BGR2GRAY);

  ROS_DEBUG("Extracting Keypoints");
  // First get the keypoints
  vector<KeyPoint> keypoints;
  keyGenerator_->detect(monoImage, keypoints);
  ROS_INFO_STREAM("Found " << keypoints.size() << " keypoints.");

  const Mat* workingImage = &image;
  if (needsMono_) {
    workingImage = &monoImage;
  }

  // Only use the keypoints in the mask
  vector<KeyPoint>* validKeypoints = &keypoints;
  vector<KeyPoint> tmpKeypoints;
  if (!mask.empty()) {
    ROS_INFO("Using a mask on the keypoints");
    validKeypoints = &tmpKeypoints;
    for (vector<KeyPoint>::const_iterator i = keypoints.begin();
         i != keypoints.end(); ++i) {
      if (mask(static_cast<int>(i->pt.y), static_cast<int>(i->pt.x))) {
        tmpKeypoints.push_back(*i);
      }
    }
  }

  // Now get all the descriptors
  ROS_INFO_STREAM("Extracting Descriptors from "
                  << keypoints.size() << " keypoints");
  wholeImageDescriptors_.reset(new DescriptorCollection());
  if (keypoints.size() > 0) {
    ExtractFromWholeImage(*workingImage, *validKeypoints,
                          wholeImageDescriptors_.get());
  } else {
    ROS_WARN("Did not find any valid keypoints for this image");
  }

  curImageData_ = image.data;
}

template <typename T>
void ImageDescriptorGenerator<T>::ExtractFromSingleMask(
    const cv::Mat& image,
    const reefbot_msgs::ImageRegion& mask,
    DescriptorCollection* collection) const {
  // Make sure that InitializeForWholeImage was called for this image
  ROS_ASSERT(image.data == curImageData_);
  ROS_ASSERT(collection);

  // Copy all the descriptors in this mask into the output
  for(typename DescriptorCollection::const_iterator descI = wholeImageDescriptors_->begin();
      descI != wholeImageDescriptors_->end(); ++descI) {
    if (MaskContainsDescriptor(mask, **descI)) {
      collection->push_back(*descI);
    }
  }
}

template <typename T>
bool ImageDescriptorGenerator<T>::MaskContainsPoint(
  const ImageRegion& mask,
  const Point2f& point) const {
  
  const sensor_msgs::RegionOfInterest& bbox = mask.bounding_box;

  // Is the point in the bounding box?
  if (((int)point.x) >= (int)bbox.x_offset &&
      ((int)point.x) < (int)(bbox.x_offset + bbox.width) &&
      ((int)point.y) >= (int)bbox.y_offset &&
      ((int)point.y) < (int)(bbox.y_offset + bbox.height)) {
    // Is the mask image empty?
    if (mask.mask.height == 0 || mask.mask.width == 0) {
      return true;
    }

    // Now check if the point is flagged in the mask image
    sensor_msgs::CvBridge bridge;
    IplImage* maskImagePtr = NULL;
    
    // TODO(mdesnoyer): This will create a copy of the data, but is
    // necessary because the interface for the bridge needs a
    // scoped_ptr and if you do it directly, the data gets deallocated
    // illegally. This might need to be fixed if things are too slow.
    if (maskCache_.get() == NULL) {
      maskCache_ = sensor_msgs::Image::Ptr(new sensor_msgs::Image(mask.mask));
    }
    try {
      maskImagePtr = bridge.imgMsgToCv(maskCache_, "mono8");
    } catch (sensor_msgs::CvBridgeException error) {
      ROS_ERROR("Problem reading the image from the message: %s",
                error.what());
      return false;
    }
    Mat maskImage(maskImagePtr);

    if (maskImage.at<uchar>(point - Point2f(bbox.x_offset, bbox.y_offset))) {
      return true;
    }
  }
  return false;
}

template <typename T>
bool ImageDescriptorGenerator<T>::MaskContainsDescriptor(
   const ImageRegion& mask,
   const ImageDescriptor<T>& descriptor) const {
  return MaskContainsPoint(mask, descriptor.point());
}

template <typename T>
void ImageDescriptorGenerator<T>::AddToCombinedMasks(
  const reefbot_msgs::ImageRegion& mask,
  Mat_<uchar>& combinedMask) const {

  const sensor_msgs::RegionOfInterest& bbox = mask.bounding_box;

  Mat_<uchar> roi(combinedMask, Rect_<int>(bbox.x_offset, bbox.y_offset,
                                           bbox.width, bbox.height));

  // Is the mask image empty, then just draw the square
  if (mask.mask.height == 0 || mask.mask.width == 0) {
    roi = 1;
    return;
  }
  
  // If the mask is more detailed, then fill it in
  sensor_msgs::CvBridge bridge;
  IplImage* maskImagePtr = NULL;
    
  // TODO(mdesnoyer): This will create a copy of the data, but is
  // necessary because the interface for the bridge needs a
  // scoped_ptr and if you do it directly, the data gets deallocated
  // illegally. This might need to be fixed if things are too slow.
  if (maskCache_.get() == NULL) {
    maskCache_ = sensor_msgs::Image::Ptr(new sensor_msgs::Image(mask.mask));
  }
  try {
    maskImagePtr = bridge.imgMsgToCv(maskCache_, "mono8");
  } catch (sensor_msgs::CvBridgeException error) {
    ROS_ERROR("Problem reading the image from the message: %s",
              error.what());
    return;
  }
  Mat maskImage(maskImagePtr);

  roi = roi | maskImage;
  
}

}

#endif 
