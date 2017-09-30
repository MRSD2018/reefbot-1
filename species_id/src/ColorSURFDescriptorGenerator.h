// Class for creating Colorized SURF Descriptors of images.
//
// This assumes that the image is in BGR format, the default for
// OpenCV. Don't ask me why that's the default, but it is.
//
// Author: Mark Desnoyer
// Date: March 2011

#ifndef _SPECIES_ID_COLOR_SURF_DESCRIPTOR_GENERATOR_H_
#define _SPECIES_ID_COLOR_SURF_DESCRIPTOR_GENERATOR_H_

#include <boost/shared_ptr.hpp>
#include "opencv2/features2d/features2d.hpp"

#include "SURFDescriptorGenerator.h"
#include "ColorConverter.h"
#include "ImageDescriptor.h"


namespace species_id {

class ColorSURFDescriptorGenerator : public SURFDescriptorGenerator {
public:
  virtual ~ColorSURFDescriptorGenerator();

  // Parameters for the SURF description.
  ColorSURFDescriptorGenerator(
    boost::shared_ptr<ColorConverter> colorConverter,
    const cv::FeatureDetector* keyGenerator,
    int nOctaves=4,
    int nOctaveLayers=2,
    bool extended=false
    );

private:
  boost::shared_ptr<ColorConverter> colorConverter_;

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

#endif // _SPECIES_ID_COLOR_SURF_DESCRIPTOR_GENERATOR_H_
