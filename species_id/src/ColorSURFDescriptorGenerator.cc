// Class for creating Colorized SIFT Descriptors of images
//
// Author: Mark Desnoyer
// Date: March 2011

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include "ColorSURFDescriptorGenerator.h"

#include "SURFDescriptor.h"

using namespace std;
using namespace boost;
using namespace cv;

namespace species_id {

ColorSURFDescriptorGenerator::ColorSURFDescriptorGenerator(
    boost::shared_ptr<ColorConverter> colorConverter,
    const FeatureDetector* keyGenerator,
    int nOctaves,
    int nOctaveLayers,
    bool extended) :
  SURFDescriptorGenerator(keyGenerator, nOctaves, nOctaveLayers, extended), 
  colorConverter_(colorConverter) {
  needsMono_ = false;
}

ColorSURFDescriptorGenerator::~ColorSURFDescriptorGenerator() {}

void ColorSURFDescriptorGenerator::ExtractFromWholeImage(
    const Mat& image,
    const vector<KeyPoint>& keypoints,
    DescriptorCollection* descriptors) const {
  ROS_ASSERT(descriptors != NULL);

  // First convert the image into the color space we need
  Mat colorImage;
  //image.convertTo(colorImage, CV_32FC3);
  colorConverter_->ConvertImage(image, &colorImage);

  // Build up the collection of descriptors to return
  for (unsigned int i = 0; i < keypoints.size(); ++i) {
    descriptors->push_back(DescriptorPtr(new SURFDescriptor(keypoints[i])));
  }

  // Run the SURF algorithm for each color, this returns
  // the descriptors in descWrapper, which is a matrix where each row
  // defines a descriptor and each column is an entry in that
  // descriptor.
  vector<Mat> splitColorImage;
  split(colorImage, splitColorImage);
  for (vector<Mat>::iterator channelI = splitColorImage.begin();
       channelI != splitColorImage.end();
       ++channelI) {

    Mat descWrapper;
    surfImpl_.compute(*channelI,
                      const_cast<vector<KeyPoint>&>(keypoints),
                      descWrapper);

    // Now extract the descriptors into the SURFDescriptor objects.
    for (unsigned int i = 0; i < keypoints.size(); ++i) {
      Mat curRow(descWrapper.row(i));
      (*descriptors)[i]->AppendVals(curRow.begin<float>(),
                                    curRow.end<float>());
    }
  }    
}

}
