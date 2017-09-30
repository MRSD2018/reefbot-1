// Class for converting images from one color space to another
//
// This default assumes that the image is in BGR format, the default for
// OpenCV. Don't ask me why that's the default, but it is.
//
// Author: Mark Desnoyer
// Date: March 2011

#include "ColorConverter.h"
#include <assert.h>

using namespace cv;

namespace species_id {

ColorConverter::~ColorConverter() {}

void ColorConverter::ConvertImage(const Mat& src, Mat* dest) {
  assert(dest != NULL);

  if (conversionCode_ == CV_COLORCVT_MAX) {
    *dest = src;
  } else {
    if (&src == dest) {
      // Same input as output matrix. This will break the OpenCV code,
      // so put a dummy in and swap afterwards.
      Mat dummy;
      cvtColor(src, dummy, conversionCode_);
      *dest = dummy;
    } else {
      cvtColor(src, *dest, conversionCode_);
    }
  }
}

}// namespace
