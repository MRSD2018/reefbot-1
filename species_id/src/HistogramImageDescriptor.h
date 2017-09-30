// An image descriptor that is a histogram of some other type of descriptors
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: Sept 2010

#ifndef _SPECIES_ID_HISTOGRAM_IMAGE_DESCRIPTOR_H__
#define _SPECIES_ID_HISTOGRAM_IMAGE_DESCRIPTOR_H__

#include <boost/shared_ptr.hpp>
#include <vector>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <deque>
#include <ros/ros.h>

#include "ImageDescriptor.h"
#include "ImageDocument.h"

namespace species_id {

class HistogramImageDescriptor : public ImageDescriptor<float> {
public:
  typedef boost::shared_ptr<HistogramImageDescriptor> Ptr;
  typedef boost::shared_ptr<HistogramImageDescriptor const> ConstPtr;

  HistogramImageDescriptor(int size)
    : ImageDescriptor<float>(), doc_(new ImageDocument()),
      descriptorSize_(size) {}

  template <class InputIterator>
  HistogramImageDescriptor(InputIterator begin, InputIterator end)
    : ImageDescriptor<float>() {
    AddVals(begin, end);
    Normalize();
  }

  virtual ~HistogramImageDescriptor();

  // Normalizes the histogram to sum to one
  void Normalize();

  virtual const float GetVal(int index) const;
  virtual void SetVal(int index, const float& val);
  void AddVal(int index, const float& val, const cv::Point2f* coord=NULL);
  virtual int DescriptorSize() const;

  // Adds this histogram descriptor to an image document
  void AddToImageDocument(ImageDocument* doc, TermID offset=0,
                          float minVal=0) const {
    ROS_ASSERT(doc != NULL);
    doc->AddTermsFromDoc(*doc_, offset, minVal);
  }

  HistogramImageDescriptor& operator*=(float val) {
    *doc_ *= val;
    return *this;
  }

  // Allows us to swap out the document and thus save a
  // copy. Invalidates this image descriptor so use with caution.
  void SwapImageDocument(boost::shared_ptr<ImageDocument>& otherDoc) {
    doc_.swap(otherDoc);
  }

private:
  // The ImageDocument 
  boost::shared_ptr<ImageDocument> doc_;

  int descriptorSize_;

  template <class InputIterator>
  void AddVals(InputIterator begin, InputIterator end) {
    int index=0;
    for (InputIterator i = begin; i != end; ++i) {
      doc_->SetTerm(index++, *i);
    }
  }
};

} // namespace

#endif // _SPECIES_ID_HISTOGRAM_IMAGE_DESCRIPTOR_H__
