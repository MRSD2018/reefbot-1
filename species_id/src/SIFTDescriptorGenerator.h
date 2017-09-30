// Class for creating SIFT Descriptors of images
//
// Author: Mark Desnoyer
// Date: June 2010

#ifndef _SPECIES_ID_SIFT_DESCRIPTOR_GENERATOR_H_
#define _SPECIES_ID_SIFT_DESCRIPTOR_GENERATOR_H_

#include "opencv2/features2d/features2d.hpp"

#include "ImageDescriptor.h"
#include "ImageDescriptorGenerator.h"


namespace species_id {

class SIFTDescriptorGenerator : public ImageDescriptorGenerator<float> {
public:
  virtual ~SIFTDescriptorGenerator();

  // Parameters for the SIFT description. See
  // http://www.vlfeat.org/api/sift_8h.html for details.
  SIFTDescriptorGenerator(
    const cv::FeatureDetector* keyGenerator,             
    double magnification=
      cv::SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(),
    bool isNormalize=true,
    bool recalculateAngles = true,
    int nOctaves=cv::SIFT::CommonParams::DEFAULT_NOCTAVES,
    int nOctaveLayers=cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
    int firstOctave=cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
    int angleMode=cv::SIFT::CommonParams::FIRST_ANGLE);

private:
  cv::SIFT siftImpl_;

  // Function defined by the subclasses that find the SIFT descriptors
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

#endif // _SPECIES_ID_SIFT_DESCRIPTOR_GENERATOR_H_
