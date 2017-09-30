// Descriptor that extracts the color at the given pixel. Chooses the
// new color space based on the colorConversionCode. These codes are
// described in the OpenCV cvtColor documentation:
// http://opencv.willowgarage.com/documentation/cpp/miscellaneous_image_transformations.html#cv-cvtcolor
//
// For example, the code are CV_RGB2HSV converts an rgb image to HSV
//
// The resulting features will always be floating point between 0 and 1.0
//
// Author: Mark Desnoyer
// Date: July 2010

#ifndef _SPECIES_ID_HSV_DESCRIPTOR_GENERATOR_H_
#define _SPECIES_ID_HSV_DESCRIPTOR_GENERATOR_H_

#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "base/StringPiece.h"
#include "ImageDescriptor.h"
#include "ImageDescriptorGenerator.h"

namespace species_id {

class ColorDescriptorGenerator : public ImageDescriptorGenerator<float> {
public:
  // Constant to use if color conversion shouldn't happen
  static const int NO_COLOR_CVT = -1;

  virtual ~ColorDescriptorGenerator();

  // Constructor that identifies the keypoint generator and the color
  // conversion code.
  ColorDescriptorGenerator(const cv::FeatureDetector* keyGenerator,
                           int colorConversionCode)
    : ImageDescriptorGenerator<float>(keyGenerator),
      colorConversionCode_(colorConversionCode)
  { needsMono_ = false; }

  // Converts a string opencv color conversion code to it's actual
  // integer code
  static int ConversionCodeStringToInt(const StringPiece& strCode);

private:
  int colorConversionCode_;

  virtual void ExtractFromWholeImage(
    const cv::Mat& image,
    const std::vector<cv::KeyPoint>& keypoints,
    DescriptorCollection* descriptors) const;

  DISALLOW_EVIL_CONSTRUCTORS(ColorDescriptorGenerator);
};

}

#endif
