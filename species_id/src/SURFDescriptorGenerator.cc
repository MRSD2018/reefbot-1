// Class for creating SIFT Descriptors of images
//
// Author: Mark Desnoyer
// Date: June 2010

#include <ros/ros.h>
#include "SURFDescriptorGenerator.h"

#include "SURFDescriptor.h"

using namespace std;
using cv::KeyPoint;

namespace species_id {

SURFDescriptorGenerator::SURFDescriptorGenerator(
    const cv::FeatureDetector* keyGenerator,
    int nOctaves,
    int nOctaveLayers,
    bool extended) :
  ImageDescriptorGenerator<float>(keyGenerator), 
  surfImpl_(nOctaves, nOctaveLayers, extended) {}

SURFDescriptorGenerator::~SURFDescriptorGenerator() {}

void SURFDescriptorGenerator::ExtractFromWholeImage(
    const cv::Mat& image,
    const vector<KeyPoint>& keypoints,
    DescriptorCollection* descriptors) const {
  ROS_ASSERT(descriptors != NULL);

  // Run the SURF algorithm, this returns the descriptors in
  // descWrapper, which is a matrix where each row defines a
  // descriptor and each column is an entry in that descriptor.
  cv::Mat descWrapper;
  surfImpl_.compute(image,
                    const_cast<vector<KeyPoint>&>(keypoints),
                    descWrapper);

  // Now extract the descriptors into SURFDescriptor objects.
  for (unsigned int i = 0; i < keypoints.size(); ++i) {
    cv::Mat curRow(descWrapper.row(i));

    DescriptorPtr curDescriptor(new SURFDescriptor(
      keypoints[i], curRow.begin<float>(), curRow.end<float>()));
    descriptors->push_back(curDescriptor);
  }
         
}

}
