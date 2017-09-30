// Classes that can be used to calculate the similarity between two
// ImageDocuments.
#ifndef __SPECIES_ID_DOC_SIMILARITY_CALCULATOR_H__
#define __SPECIES_ID_DOC_SIMILARITY_CALCULATOR_H__

#include "ImageDocument.h"

namespace species_id {

// Abstract class
class DocSimilarityCalculator {
public:
  virtual ~DocSimilarityCalculator();

  // Calculates the similarity between two documents.
  // The documents must already be multiplied by the idf vector
  virtual double CalcDocumentSimilarity(const ImageDocument& a,
                                        const ImageDocument& b) const=0;

};

class CosineSimilarity : public DocSimilarityCalculator {
public:
  CosineSimilarity() {}
  virtual ~CosineSimilarity();
  
  virtual double CalcDocumentSimilarity(const ImageDocument& a,
                                        const ImageDocument& b) const;
};

// Abstract class
class ReweightDocSimilarity {
public:
  virtual ~ReweightDocSimilarity();

  virtual double CalcNewSimilarity(const ImageDocument& a,
                                   const ImageDocument& b,
                                   double oldSimilarity) const=0;
};

// Reweights the score by taking the common terms in the document and
// trying to find the perspective transform between them.
//
// The new score becomes the number of inlier pairs plus the old
// similarity score.
//
// An inlier is one where the reprojection error of the point is less
// than inlierThresh
class PerspectiveGeometricReweight : public ReweightDocSimilarity {
public:
  PerspectiveGeometricReweight(double inlierThresh)
    : ReweightDocSimilarity(), inlierThresh_(inlierThresh*inlierThresh) {}
  virtual ~PerspectiveGeometricReweight();

  virtual double CalcNewSimilarity(const ImageDocument& a,
                                   const ImageDocument& b,
                                   double oldSimilarity) const;

private:
  double inlierThresh_;
};

} // namespace


#endif // __SPECIES_ID_DOC_SIMILARITY_CALCULATOR_H__
