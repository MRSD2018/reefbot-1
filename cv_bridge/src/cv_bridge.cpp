/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <boost/make_shared.hpp>

namespace enc = sensor_msgs::image_encodings;

namespace cv_bridge {

int getCvType(const std::string& encoding)
{
  // Check for the most common encodings first
  if (encoding == enc::BGR8)   return CV_8UC3;
  if (encoding == enc::MONO8)  return CV_8UC1;
  if (encoding == enc::RGB8)   return CV_8UC3;
  if (encoding == enc::MONO16) return CV_16UC1;
  if (encoding == enc::BGR16)  return CV_16UC3;
  if (encoding == enc::RGB16)  return CV_16UC3;
  if (encoding == enc::BGRA8)  return CV_8UC4;
  if (encoding == enc::RGBA8)  return CV_8UC4;
  if (encoding == enc::BGRA16) return CV_16UC4;
  if (encoding == enc::RGBA16) return CV_16UC4;

  // All the generic types with no channels
  if (encoding == enc::TYPE_8U) return CV_8U;
  if (encoding == enc::TYPE_8S) return CV_8S;
  if (encoding == enc::TYPE_16U) return CV_16U;
  if (encoding == enc::TYPE_16S) return CV_16S;
  if (encoding == enc::TYPE_32S) return CV_32S;
  if (encoding == enc::TYPE_32F) return CV_32F;
  if (encoding == enc::TYPE_64F) return CV_64F;

  // For bayer, return one-channel
  if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
  if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
  if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
  if (encoding == enc::BAYER_GRBG8) return CV_8UC1;
  if (encoding == enc::BAYER_RGGB16) return CV_16UC1;
  if (encoding == enc::BAYER_BGGR16) return CV_16UC1;
  if (encoding == enc::BAYER_GBRG16) return CV_16UC1;
  if (encoding == enc::BAYER_GRBG16) return CV_16UC1;

  // Check all the generic content encodings
#define CHECK_ENCODING(code)                            \
  if (encoding == enc::TYPE_##code) return CV_##code    \
  /***/
#define CHECK_CHANNEL_TYPE(t)                   \
  CHECK_ENCODING(t##1);                         \
  CHECK_ENCODING(t##2);                         \
  CHECK_ENCODING(t##3);                         \
  CHECK_ENCODING(t##4);                         \
  /***/

  CHECK_CHANNEL_TYPE(8UC);
  CHECK_CHANNEL_TYPE(8SC);
  CHECK_CHANNEL_TYPE(16UC);
  CHECK_CHANNEL_TYPE(16SC);
  CHECK_CHANNEL_TYPE(32SC);
  CHECK_CHANNEL_TYPE(32FC);
  CHECK_CHANNEL_TYPE(64FC);

#undef CHECK_CHANNEL_TYPE
#undef CHECK_ENCODING

  throw Exception("Unrecognized image encoding [" + encoding + "]");
}

/// @cond DOXYGEN_IGNORE

enum Format { INVALID = -1, GRAY = 0, RGB, BGR, RGBA, BGRA };

Format getFormat(const std::string& encoding)
{
  if (encoding == enc::BGR8)   return BGR;
  if (encoding == enc::MONO8)  return GRAY;
  if (encoding == enc::RGB8)   return RGB;
  if (encoding == enc::MONO16) return GRAY;
  if (encoding == enc::BGR16)  return BGR;
  if (encoding == enc::RGB16)  return RGB;
  if (encoding == enc::BGRA8)  return BGRA;
  if (encoding == enc::RGBA8)  return RGBA;
  if (encoding == enc::BGRA16) return BGRA;
  if (encoding == enc::RGBA16) return RGBA;

  // We don't support conversions to/from other types
  return INVALID;
}

static const int SAME_FORMAT = -1;

int getConversionCode(Format src_format, Format dst_format)
{
  static const int CONVERSION_CODES[] = { SAME_FORMAT,
                                          CV_GRAY2RGB,
                                          CV_GRAY2BGR,
                                          CV_GRAY2RGBA,
                                          CV_GRAY2BGRA,
                                          CV_RGB2GRAY,
                                          SAME_FORMAT,
                                          CV_RGB2BGR,
                                          CV_RGB2RGBA,
                                          CV_RGB2BGRA,
                                          CV_BGR2GRAY,
                                          CV_BGR2RGB,
                                          SAME_FORMAT,
                                          CV_BGR2RGBA,
                                          CV_BGR2BGRA,
                                          CV_RGBA2GRAY,
                                          CV_RGBA2RGB,
                                          CV_RGBA2BGR,
                                          SAME_FORMAT,
                                          CV_RGBA2BGRA,
                                          CV_BGRA2GRAY,
                                          CV_BGRA2RGB,
                                          CV_BGRA2BGR,
                                          CV_BGRA2RGBA,
                                          SAME_FORMAT };
  return CONVERSION_CODES[src_format*5 + dst_format];
}

// Determines the encoding string from the matrix type
void GetEncodingOfMat(const cv::Mat& mat, std::string* encoding) {
  ROS_ASSERT(encoding);
  if (mat.dims == 2) {

    // Check for all the generic type codes
#define CHECK_CV_TYPE(code)                            \
    if (mat.type() == CV_##code) *encoding = enc::TYPE_##code; 
  /***/
#define CHECK_CV_CHANNEL_TYPE(t)                   \
  CHECK_CV_TYPE(t##1);                         \
  CHECK_CV_TYPE(t##2);                         \
  CHECK_CV_TYPE(t##3);                         \
  CHECK_CV_TYPE(t##4);                         \
  /***/

    CHECK_CV_CHANNEL_TYPE(8UC);
    CHECK_CV_CHANNEL_TYPE(8SC);
    CHECK_CV_CHANNEL_TYPE(16UC);
    CHECK_CV_CHANNEL_TYPE(16SC);
    CHECK_CV_CHANNEL_TYPE(32SC);
    CHECK_CV_CHANNEL_TYPE(32FC);
    CHECK_CV_CHANNEL_TYPE(64FC);

#undef CHECK_CV_TYPE
#undef CHECK_CV_CHANNEL_TYPE

  } else if (mat.dims > 2) {
    // It's a multiple dimensional array so the encoding can only be
    // of basic types.
    if (mat.depth() == CV_8U) *encoding = enc::TYPE_8U;
    if (mat.depth() == CV_8S) *encoding = enc::TYPE_8S;
    if (mat.depth() == CV_16U) *encoding = enc::TYPE_16U;
    if (mat.depth() == CV_16S) *encoding = enc::TYPE_16S;
    if (mat.depth() == CV_32S) *encoding = enc::TYPE_32S;
    if (mat.depth() == CV_32F) *encoding = enc::TYPE_32F;
    if (mat.depth() == CV_64F) *encoding = enc::TYPE_64F;
  }
}

// Internal, used by toCvCopy and cvtColor
CvImagePtr toCvCopyImpl(const cv::Mat& source,
                        const roslib::Header& src_header,
                        const std::string& src_encoding,
                        const std::string& dst_encoding)
{
  /// @todo Handle endianness - e.g. 16-bit dc1394 camera images are big-endian
  
  // Copy metadata
  CvImagePtr ptr = boost::make_shared<CvImage>();
  ptr->header = src_header;
  
  // Copy to new buffer if same encoding requested
  if (dst_encoding.empty() || dst_encoding == src_encoding)
  {
    ptr->encoding = src_encoding;
    source.copyTo(ptr->image);
  }
  else
  {
    // Convert the source data to the desired encoding
    Format src_format = getFormat(src_encoding);
    Format dst_format = getFormat(dst_encoding);
    if (src_format == INVALID || dst_format == INVALID)
      throw Exception("Unsupported conversion from [" + src_encoding +
                      "] to [" + dst_encoding + "]");

    int conversion_code = getConversionCode(src_format, dst_format);
    if (conversion_code == SAME_FORMAT)
    {
      // Same number of channels, but different bit depth
      double alpha = 1.0;
      int src_depth = enc::bitDepth(src_encoding);
      int dst_depth = enc::bitDepth(dst_encoding);
      // Do scaling between CV_8U [0,255] and CV_16U [0,65535] images.
      if (src_depth == 8 && dst_depth == 16)
        alpha = 65535. / 255.;
      else if (src_depth == 16 && dst_depth == 8)
        alpha = 255. / 65535.;
      source.convertTo(ptr->image, getCvType(dst_encoding), alpha);
    }
    else
    {
      // Perform color conversion
      cv::cvtColor(source, ptr->image, conversion_code);
    }
    ptr->encoding = dst_encoding;
  }

  return ptr;
}

// Used by toCvCopy and cvtType
CvMultiMatPtr toCvCopyMultiMatImpl(const cv::Mat& source,
                                   const roslib::Header& src_header,
                                   const std::string& src_encoding,
                                   const std::string& dst_encoding)
{
  CvMultiMatPtr ptr = boost::make_shared<CvMultiMat>();
  ptr->header = src_header;

  // Copy to a new buffer if the encoding is the same
  if (dst_encoding.empty() || dst_encoding == src_encoding)
  {
    ptr->encoding = src_encoding;
    source.copyTo(ptr->mat);
  }
  else
  {
    // Different data type so convert
    source.convertTo(ptr->mat, getCvType(dst_encoding));
    ptr->encoding = dst_encoding;
  }

  return ptr;
}

/// @endcond

sensor_msgs::ImagePtr CvImage::toImageMsg() const
{
  sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  toImageMsg(*ptr);
  return ptr;
}

void CvImage::toImageMsg(sensor_msgs::Image& ros_image) const
{
  ros_image.header = header;
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  ros_image.encoding = encoding;
  ros_image.is_bigendian = false;
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  ros_image.data.resize(size);

  if (image.isContinuous())
  {
    memcpy((char*)(&ros_image.data[0]), image.data, size);
  }
  else
  {
    // Copy by row by row
    uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
    uchar* cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i)
    {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}

sensor_msgs::MatNDPtr CvMultiMat::toMsg() const
{
  sensor_msgs::MatNDPtr ptr = boost::make_shared<sensor_msgs::MatND>();
  toMsg(*ptr);
  return ptr;
}

void CvMultiMat::toMsg(sensor_msgs::MatND& msg) const {
  msg.header = header;
  size_t totalSize = 1;
  for (int i = 0; i < mat.dims; ++i) {
    msg.sizes.push_back(mat.size[i]);
    totalSize *= mat.size[i];
  }
  totalSize *= mat.elemSize();
  if (encoding == std::string()) {
    GetEncodingOfMat(mat, &msg.encoding);
  } else {
    msg.encoding = encoding;
  }
  msg.is_bigendian = false;
  msg.data.resize(totalSize);
  if (mat.isContinuous())
  {
    memcpy((char*)(&msg.data[0]), mat.data, totalSize);
  }
  else if (totalSize > 0)
  {
    // TODO: handle non-continuous data using the iterators. Something
    // similar to the following
    // for (cv::Mat::const_iterator matI = mat.begin(), i = 0;
    //     matI != mat.end(); ++matI, ++i) {
    //  memcpy((char*)(msg.data[i * mat.elemSize()]), &(*matI),
    //         mat.elemSize());
    throw cv_bridge::Exception("The multi dimensional matrix is not continuous. Packing it into a message has not been implemented");
  }
}



// Deep copy data, returnee is mutable
CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
                    const std::string& encoding)
{
  return toCvCopy(*source, encoding);
}

CvImagePtr toCvCopy(const sensor_msgs::Image& source,
                    const std::string& encoding)
{
  // Construct matrix pointing to source data
  int source_type = getCvType(source.encoding);
  const cv::Mat tmp((int)source.height, (int)source.width, source_type,
                    const_cast<uint8_t*>(&source.data[0]), (size_t)source.step);

  return toCvCopyImpl(tmp, source.header, source.encoding, encoding);
}

CvMultiMatPtr toCvCopy(const sensor_msgs::MatNDConstPtr& source,
                       const std::string& encoding)
{
  return toCvCopy(*source, encoding);
}

CvMultiMatPtr toCvCopy(const sensor_msgs::MatND& source,
                       const std::string& encoding) 
{
  // Construct matrix pointing to source data
  int source_type = getCvType(source.encoding);
  const cv::Mat tmp(source.sizes.size(), &source.sizes[0], source_type,
                    const_cast<uint8_t*>(&source.data[0]));

  return toCvCopyMultiMatImpl(tmp, source.header, source.encoding, encoding);
}

// Share const data, returnee is immutable
CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source,
                          const std::string& encoding)
{
  return toCvShare(*source, source, encoding);
}

CvMultiMatConstPtr toCvShare(const sensor_msgs::MatNDConstPtr& source,
                             const std::string& encoding)
{
  return toCvShare(*source, source, encoding);
}

CvImageConstPtr toCvShare(const sensor_msgs::Image& source,
                          const boost::shared_ptr<void const>& tracked_object,
                          const std::string& encoding)
{
  if (!encoding.empty() && source.encoding != encoding)
    return toCvCopy(source, encoding);

  CvImagePtr ptr = boost::make_shared<CvImage>();
  ptr->header = source.header;
  ptr->encoding = source.encoding;
  ptr->tracked_object_ = tracked_object;
  int type = getCvType(source.encoding);
  ptr->image = cv::Mat(source.height, source.width, type,
                       const_cast<uchar*>(&source.data[0]), source.step);
  return ptr;
}

CvMultiMatConstPtr toCvShare(const sensor_msgs::MatND& source,
                             const boost::shared_ptr<void const>& tracked_object,
                             const std::string& encoding)
{
  CvMultiMatPtr ptr = boost::make_shared<CvMultiMat>();
  ptr->header = source.header;
  ptr->encoding = source.encoding;
  ptr->tracked_object_ = tracked_object;

  if (source.sizes.size() == 0) {
    ptr->mat = cv::Mat();
    return ptr;
  }

  if (!encoding.empty() && source.encoding != encoding)
    return toCvCopy(source, encoding);

  int type = getCvType(source.encoding);
  ptr->mat = cv::Mat(source.sizes.size(), &source.sizes[0], type,
                     const_cast<uint8_t*>(&source.data[0]));
  return ptr;
}

CvImagePtr cvtColor(const CvImageConstPtr& source,
                    const std::string& encoding)
{
  return toCvCopyImpl(source->image, source->header, source->encoding, encoding);
}

CvMultiMatPtr cvtType(const CvMultiMatConstPtr& source,
                      const std::string& encoding)
{
  return toCvCopyMultiMatImpl(source->mat, source->header, source->encoding,
                              encoding);
}

} //namespace cv_bridge
