// Class for creating SIFT Descriptors of images
//
// Author: Mark Desnoyer
// Date: June 2010

#ifndef _SPECIES_ID_SURF_DESCRIPTOR_GENERATOR_H_
#define _SPECIES_ID_SURF_DESCRIPTOR_GENERATOR_H_

#include "opencv2/features2d/features2d.hpp"

#include "ImageDescriptor.h"
#include "ImageDescriptorGenerator.h"


namespace species_id {

class SURFDescriptorGenerator : public ImageDescriptorGenerator<float> {
public:
  virtual ~SURFDescriptorGenerator();

  // Parameters for the SURF description.
  SURFDescriptorGenerator(
    const cv::FeatureDetector* keyGenerator,
    int nOctaves=4,
    int nOctaveLayers=2,
    bool extended=false
    );

protected:
  cv::SurfDescriptorExtractor surfImpl_;

  // Function defined by the subclasses that find the SURF descriptors
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
    DescriptorCollection* descriptors) const; 

};

}

#endif // _SPECIES_ID_SURF_DESCRIPTOR_GENERATOR_H_
