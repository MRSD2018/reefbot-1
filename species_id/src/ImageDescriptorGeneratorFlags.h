// Defines a bunch of command line flags that can be used by various
// processes that calculate descriptors of images.
//
// Author: Mark Desnoyer(markd@cmu.edu)
// Date: June 30, 2010

#ifndef _SPECIES_ID_IMAGE_DESCRIPTOR_GENERATOR_FLAGS_H_
#define _SPECIES_ID_IMAGE_DESCRIPTOR_GENERATOR_FLAGS_H_

#include <gflags/gflags.h>

#include "ImageDescriptorGenerator.h"
#include "opencv2/features2d/features2d.hpp"

// Function that ensures that only one *_detector and *_descriptor
// flag are specified. Must be called after
// google::ParseCommandLineFlags has been run.
void VerifyDescriptorFlags();

// Based on these flags, choose the Detector to use. Caller must take
// ownership of the new object created.
cv::FeatureDetector* ChooseFeatureDetector();

species_id::ImageDescriptorGenerator<float>* ChooseImageDescriptor(
  const cv::FeatureDetector* detector);

#endif
