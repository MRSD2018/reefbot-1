// An image descriptor that is a histogram of some other type of descriptors
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: Sept 2010

#include "HistogramImageDescriptor.h"
#include "ImageDescriptor-Inl.h"

namespace species_id {

HistogramImageDescriptor::~HistogramImageDescriptor() {}

const float HistogramImageDescriptor::GetVal(int index) const {
  return doc_->GetVal(index);
}

void HistogramImageDescriptor::SetVal(int index, const float& val) {
  doc_->SetTerm(index, val);
}

void HistogramImageDescriptor::AddVal(int index, const float& val,
                                      const cv::Point2f* coord) {
  if (coord != NULL) {
    doc_->AddTerm(index, val, coord->x, coord->y);
  } else {
    doc_->AddTerm(index, val);
  }
}

int HistogramImageDescriptor::DescriptorSize() const {
  return descriptorSize_;
}

void HistogramImageDescriptor::Normalize() {
  doc_->Normalize();
}

}
