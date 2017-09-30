// Given an image, creates an ImageDocument based on multiple
// HistogramImageDescriptors.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: Oct 2010

#include "MultiHistogramDocGenerator.h"
#include "HistogramDescriptorGenerator-Inl.h"
#include "ros/assert.h"
#include <limits>

using namespace cv;
using namespace boost;
using namespace std;

namespace species_id {

void MultiHistogramDocGenerator::CreateDocsUsingMasks(
    const Mat& image,
    const vector<reefbot_msgs::ImageRegion>& masks,
    vector<shared_ptr<ImageDocument> >* documents) const {
  ROS_ASSERT(documents != NULL);

  // Load up the documents to return
  for (int i = 0; i < masks.size(); i++) {
    documents->push_back(shared_ptr<ImageDocument>(new ImageDocument()));
  }

  TermID idStart = 0;

  for (vector<GeneratorParams>::const_iterator params =
         generators_.begin();
       params != generators_.end();
       ++params) {
    if (fabs(params->weight) < 10*numeric_limits<float>::epsilon()) {
      // This generator is zeroed out
      continue;
    }

    vector<shared_ptr<HistogramDescriptorGenerator<float>::HistDescriptorCollection> >
        histogramContainer;

    params->generator->ExtractUsingMasks(image, masks,
                                         &histogramContainer);

    int histogramSize = 0;

    for (int i = 0; i < histogramContainer.size(); i++) {
      ROS_ASSERT((*histogramContainer[i]).size() == 1);

      HistogramImageDescriptor::Ptr histogram =
        (*histogramContainer[i])[0];

      (*histogram) *= params->weight;

      histogram->AddToImageDocument(((*documents)[i]).get(), idStart,
                                    params->thresh);
      histogramSize = histogram->DescriptorSize();
    }
    idStart += histogramSize;
  }

  // Normalize the documents
  for (vector<shared_ptr<ImageDocument> >::iterator docI = 
         documents->begin(); 
       docI != documents->end();
       docI++) {
    (*docI)->Normalize();
  }
}

void MultiHistogramDocGenerator::InitializeForWholeImage(
  const cv::Mat& image, const cv::Mat_<uint8>& mask) {
  for (vector<GeneratorParams>::iterator params =
         generators_.begin();
       params != generators_.end();
       ++params) {
    if (fabs(params->weight) < 10*numeric_limits<float>::epsilon()) {
      // This generator is zeroed out
      continue;
    }

    params->generator->InitializeForWholeImage(image, mask);
  }
}

shared_ptr<ImageDocument>
MultiHistogramDocGenerator::CreateDocFromSingleMask(
  const cv::Mat& image,
  const reefbot_msgs::ImageRegion& mask) const {
  shared_ptr<ImageDocument> retval;
  
  TermID idStart = 0;
  for (vector<GeneratorParams>::const_iterator params =
         generators_.begin();
       params != generators_.end();
       ++params) {
    if (fabs(params->weight) < 10*numeric_limits<float>::epsilon()) {
      // This generator is zeroed out
      continue;
    }

    HistogramImageDescriptor::Ptr curHist = 
      params->generator->ExtractHistogramFromSingleMask(image, mask);

    (*curHist) *= params->weight;

    if (retval.get() == NULL) {
      curHist->SwapImageDocument(retval);
      retval->RemoveTermsBelowThresh(params->thresh);
    } else {
      curHist->AddToImageDocument(retval.get(), idStart, params->thresh);
    }

    idStart += curHist->DescriptorSize();
  }

  return retval;
}

} // namespace
