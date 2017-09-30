// Class for creating SIFT Descriptors of images
//
// Author: Mark Desnoyer
// Date: June 2010

#include <ros/ros.h>
#include "SIFTDescriptorGenerator.h"

#include "SIFTDescriptor.h"

using namespace std;
using cv::KeyPoint;

namespace species_id {

SIFTDescriptorGenerator::SIFTDescriptorGenerator(
    const cv::FeatureDetector* keyGenerator,
    double magnification,
    bool isNormalize,
    bool recalculateAngles,
    int nOctaves,
    int nOctaveLayers,
    int firstOctave,
    int angleMode) :
  ImageDescriptorGenerator<float>(keyGenerator), 
  siftImpl_(magnification, isNormalize, recalculateAngles, nOctaves,
            nOctaveLayers, firstOctave, angleMode) {}

SIFTDescriptorGenerator::~SIFTDescriptorGenerator() {}

void SIFTDescriptorGenerator::ExtractFromWholeImage(
    const cv::Mat& image,
    const vector<KeyPoint>& keypoints,
    DescriptorCollection* descriptors) const {
  ROS_ASSERT(descriptors != NULL);

  // Run the SIFT algorithm, this returns the descriptors in
  // descWrapper, which is a matrix where each row defines a
  // descriptor and each column is an entry in that descriptor.
  cv::Mat descWrapper;
  siftImpl_(image, cv::Mat(), const_cast<vector<KeyPoint>&>(keypoints),
            descWrapper, true);

  // Now extract the descriptors into SIFTDescriptor objects.
  for (unsigned int i = 0; i < keypoints.size(); ++i) {
    cv::Mat curRow(descWrapper.row(i));

    DescriptorPtr curDescriptor(new SIFTDescriptor(
      keypoints[i], curRow.begin<float>(), curRow.end<float>()));
    descriptors->push_back(curDescriptor);
  }
         
}

}
