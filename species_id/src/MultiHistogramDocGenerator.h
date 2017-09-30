// Given an image, creates an ImageDocument based on multiple
// HistogramImageDescriptors.
//
// To use this class, you must create an object, add histogram types,
// then pass images through it to generate documents.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: Oct 2010

#ifndef _SPECIES_ID_MULTI_HISTOGRAM_DOC_GENERATOR_H__
#define _SPECIES_ID_MULTI_HISTOGRAM_DOC_GENERATOR_H__

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include "opencv2/core/core.hpp"
#include <vector>
#include "base/StringPiece.h"
#include "HistogramDescriptorGenerator.h"
#include "ImageDocument.h"
#include "reefbot_msgs/ImageRegion.h"

namespace species_id {

struct GeneratorParams {
  GeneratorParams(HistogramDescriptorGenerator<float>* gen,
                  float t,
                  float w)
    : generator(gen), thresh(t), weight(w) {}

  boost::shared_ptr<HistogramDescriptorGenerator<float> >
    generator;
  float thresh;
  float weight;
};

class MultiHistogramDocGenerator {
public:
  typedef std::vector<reefbot_msgs::ImageRegion::ConstPtr> MaskCollection;

  MultiHistogramDocGenerator() {}
  ~MultiHistogramDocGenerator() {}

  // Registers a type of histogram that should be included for this image.
  //
  // Inputs:
  // generator - Generator that will create the histograms
  // from the image. Takes ownership of the object.
  //
  // entryThresh - An entry in the histogram must be greater than this
  // threshold to be included in the document. A higher value will
  // give less accurate results, but will be faster.
  //
  // weight - Weighting for this type of histogram if you're combining
  // with other types.
  void AddHistogramType(HistogramDescriptorGenerator<float>* generator,
                        float entryThresh,
                        float weight) {
    generators_.push_back(GeneratorParams(generator, entryThresh, weight));
  }
    
  // Creates an ImageDocument from a portion of the image and the
  // histograms that have been loaded up using AddHistogramType.
  //
  // Inputs:
  // image - Full image frame
  // masks - Defines the portion of the frame we care about
  //
  // Outputs:
  // documents - List of documents, one per mask
  void CreateDocsUsingMasks(
    const cv::Mat& image,
    const std::vector<reefbot_msgs::ImageRegion>& masks,
    std::vector<boost::shared_ptr<ImageDocument> >* documents) const;

  // Instead of using a collection of masks for a given image, we can
  // call each mask independently. InitializeForWholeImage must be
  // called first
  //
  // Inputs:
  // image - The image to extract the descriptors from
  // mask - Mask to get the descriptor of
  //
  // Outputs:
  // The document for this mask
  boost::shared_ptr<ImageDocument> CreateDocFromSingleMask(
    const cv::Mat& image,
    const reefbot_msgs::ImageRegion& mask) const;

  // Initializes the entire image so that calls to CreateDocFromSingleMask
  // can execute efficiently.
  // 
  // Inputs:
  // image - Image to initialize for.
  // mask - Optional boolean mask that specifies locations in the
  // image where it is valid to have a descriptors. N.B. If this is
  // used, it is the user's responsibility to make sure that only the
  // regions specified will be requested. No error will be reported if
  // you use it wrong.
  void InitializeForWholeImage(const cv::Mat& image,
                               const cv::Mat_<uint8>& mask=cv::Mat_<uint8>());

private:
  std::vector<GeneratorParams> generators_;
};

} // Namespace

#endif // _SPECIES_ID_MULTI_HISTOGRAM_DOC_GENERATOR_H__
