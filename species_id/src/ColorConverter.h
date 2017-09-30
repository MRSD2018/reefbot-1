// Class for converting images from one color space to another
//
// This default assumes that the image is in BGR format, the default for
// OpenCV. Don't ask me why that's the default, but it is.
//
// Author: Mark Desnoyer
// Date: March 2011

#ifndef _SPECIES_ID_COLOR_CONVERTER_H__
#define _SPECIES_ID_COLOR_CONVERTER_H__

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

namespace species_id {

class ColorConverter {
public:

  // Creates a color converter using the default OpenCV conversion codes in cvtColor
  //
  // If using CV_COLORCVT_MAX, then no change will be made to the image.
  // Remember that OpenCV puts the images in BGR format, not RGB
  ColorConverter(int conversionCode = CV_COLORCVT_MAX)
    : conversionCode_(conversionCode) {}
  
  virtual ~ColorConverter();

  // Converts the src image into a different color space and puts the result in dest
  virtual void ConvertImage(const cv::Mat& src, cv::Mat* dest);

private:
  int conversionCode_;
};

// Converts to the opponent color space where:
//
// O(1) = (R - G) / sqrt(2)
// O(2) = (R + G - 2B) / sqrt(6)
// O(3) = (R + G + B) / sqrt(3)
//
// Assumes that the image is a standard OpenCV image in BGR format
//
// Also, offsets the first two channels so that they are in the middle
// of the range. (i.e. 0 is at 128 for a uchar image)
//
// Typename is the type of your image, like char, int, etc..
template<typename T>
class OpponentColorConverter: public ColorConverter {

public:
  OpponentColorConverter() : ColorConverter() {}

  virtual ~OpponentColorConverter();
  virtual void ConvertImage(const cv::Mat& src, cv::Mat* dest);
};

// Converts to the c invariant opponent color space where:
//
// The opponent color space is defined by:
// O(1) = (R - G) / sqrt(2)
// O(2) = (R + G - 2B) / sqrt(6)
// O(3) = (R + G + B) / sqrt(3)
//
// And the c - invariant is defined as:
//
// C(1) = O(1) / O(3)
// C(2) = O(2) / O(3)
// C(3) = O(3)
//
// Assumes that the image is a standard OpenCV image in BGR format
//
// Also, offsets the first two channels so that they are in the middle
// of the range. (i.e. 0 is at 128 for a uchar image). You really
// should use a double though.
//
// Typename is the type of your image, like char, int, etc..
template<typename T>
class CInvariantColorConverter: public ColorConverter {

public:
  CInvariantColorConverter() : ColorConverter() {}

  virtual ~CInvariantColorConverter();
  virtual void ConvertImage(const cv::Mat& src, cv::Mat* dest);
};


} // Namespace

#endif // _SPECIES_ID_COLOR_CONVERTER_H__
