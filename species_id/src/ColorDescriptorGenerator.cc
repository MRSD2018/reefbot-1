
#include "ColorDescriptorGenerator.h"

#include "opencv2/imgproc/types_c.h"
#include <ros/ros.h>

#include "ImageDescriptor-Inl.h"
#include "ImageDescriptorGenerator-Inl.h"

using namespace cv;
using namespace std;

namespace species_id {

ColorDescriptorGenerator::~ColorDescriptorGenerator() {}

void ColorDescriptorGenerator::ExtractFromWholeImage(
    const Mat& image,
    const vector<KeyPoint>& keypoints,
    DescriptorCollection* descriptors) const {
  // TODO(mdesnoye) this routine copies the data in image because it
  // is easier to write the code. This should be fixed if things are
  // slow
  int srcChannels = 3;
  switch (colorConversionCode_) {
  case CV_GRAY2BGR:
  case CV_GRAY2BGRA:
  case CV_GRAY2BGR565:
  case CV_GRAY2BGR555:
  case CV_BayerBG2BGR:
  case CV_BayerGB2BGR:
  case CV_BayerRG2BGR:
  case CV_BayerGR2BGR:
    srcChannels = 1;
  }

  if (colorConversionCode_ != NO_COLOR_CVT && 
      image.channels() != srcChannels) {
    ROS_ERROR_STREAM("Image channels do not match what's needed for "
                     << "conversion. Image channels: "
                     << image.channels()
                     << " conversion code: " << colorConversionCode_);
    return;
  }

  // First convert the image to floating point between 0 and 1
  Mat modImage;
  if (image.channels() == 1) {
    image.convertTo(modImage, CV_32FC1);
  } else if (image.channels() == 3) {
    image.convertTo(modImage, CV_32FC3);
  } else {
    ROS_ERROR_STREAM("Invalid number of channels in the image: "
                     << image.channels());
    return;
  }

  // Now do the color conversion
  if (colorConversionCode_ != NO_COLOR_CVT) {
    cvtColor(modImage, modImage, colorConversionCode_);
  }

  // Finally, extract the descriptors
  for(vector<KeyPoint>::const_iterator i = keypoints.begin();
      i != keypoints.end(); ++i) {
    Point point = i->pt;
    if (modImage.channels() == 1) {
      float val = modImage.at<float>(point);
      descriptors->push_back(
        DescriptorPtr(new ImageDescriptor<float>(point, val)));
    } else if (modImage.channels() == 3) {
      const Vec3f& val = modImage.at<Vec3f>(point);
      descriptors->push_back(DescriptorPtr(
        new ImageDescriptor<float>(point, &(val.val[0]), &(val.val[3]))));
    }
  }
}

int ColorDescriptorGenerator::ConversionCodeStringToInt(
  const StringPiece& strCode) {
  int colorConversion = 0;
  if (strCode.as_string().compare("NO_COLOR_CVT")) {
    colorConversion = ColorDescriptorGenerator::NO_COLOR_CVT;
  } else if (strCode.as_string().compare("CV_BGR2BGRA")) {
    colorConversion = CV_BGR2BGRA;
  } else if (strCode.as_string().compare("CV_BGRA2BGR")) {
    colorConversion = CV_BGRA2BGR;
  } else if (strCode.as_string().compare("CV_RGB2GRAY")) {
    colorConversion = CV_RGB2GRAY;
  } else if (strCode.as_string().compare("CV_GRAY2RGB")) {
    colorConversion = CV_GRAY2RGB;
  } else if (strCode.as_string().compare("CV_RGB2XYZ")) {
    colorConversion = CV_RGB2XYZ;
  } else if (strCode.as_string().compare("CV_XYZ2RGB")) {
    colorConversion = CV_XYZ2RGB;
  } else if (strCode.as_string().compare("CV_RGB2YCrCb")) {
    colorConversion = CV_RGB2YCrCb;
  } else if (strCode.as_string().compare("CV_YCrCb2RGB")) {
    colorConversion = CV_YCrCb2RGB;
  } else if (strCode.as_string().compare("CV_RGB2Lab")) {
    colorConversion = CV_RGB2Lab;
  } else if (strCode.as_string().compare("CV_BayerBG2RGB")) {
    colorConversion = CV_BayerBG2RGB;
  } else if (strCode.as_string().compare("CV_RGB2Luv")) {
    colorConversion = CV_RGB2Luv;
  } else if (strCode.as_string().compare("CV_RGB2HLS")) {
    colorConversion = CV_RGB2HLS;
  } else if (strCode.as_string().compare("CV_HSV2RGB")) {
    colorConversion = CV_HSV2RGB;
  } else if (strCode.as_string().compare("CV_RGB2HSV")) {
    colorConversion = CV_RGB2HSV;
  } else if (strCode.as_string().compare("CV_Lab2RGB")) {
    colorConversion = CV_Lab2RGB;
  } else if (strCode.as_string().compare("CV_Luv2RGB")) {
    colorConversion = CV_Luv2RGB;
  } else if (strCode.as_string().compare("CV_HLS2RGB")) {
    colorConversion = CV_HLS2RGB;
  } else {
    ROS_ERROR_STREAM("Invalid Color Conversion. "
                     << "You may need to add that option to the code yourself. "
                     << strCode.as_string());
  }
  return colorConversion;
}

}
