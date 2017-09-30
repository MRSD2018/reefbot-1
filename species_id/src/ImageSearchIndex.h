// An image search index. Each document is specified as a list of
// "terms" with optional (x,y) coordinates. Terms are specified by ID
// so some other object is in charge of assigning the IDs and tracking
// a dictionary. This class implements an inverted index and scoring
// routine to return the best matching image documents in the index.
//
// This class does not build an incremental index for searching, so,
// if you want a new document to be searchable, you need to rebuild
// the entire index using BuildIndex()
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: July 2010

#ifndef _SPECIES_ID_IMAGE_SEARCH_INDEX_H__
#define _SPECIES_ID_IMAGE_SEARCH_INDEX_H__

#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <gflags/gflags.h>
#include <math.h>
#include <stdint.h>
#include <vector>

#include "ImageDocument.h"
#include "SearchResult.h"
#include "DocSimilarityCalculator.h"

// TODO(mdesnoyer) implement this functionality
// DEFINE_double(max_search_time, INFINITY, "Maximum amount of time to search for possible matches. If the search is not complete after this amount of time, the best results found will be returned")

namespace species_id {

class ImageSearchIndex {
public:
  // Build an index based on the maximum number of terms in the dictionary
  ImageSearchIndex();

  // Adds a document to the index but does not make it
  // searchable. After all the documents have been added, call
  // BuildIndex to make them searchable.
  //
  // Returns a pointer to the document just added
  ImageDocument* AddDocument(const ImageDocument& doc);
  ImageDocument* AddDocument(boost::shared_ptr<ImageDocument> doc);

  // Builds the inverse index from all the documents in the
  // collection. This enables the search.
  void BuildIndex();

  // Searches for docs in the index similar to the parameter.
  //
  // Inputs:
  // doc - Document to search for somehting similar to
  //
  // Outputs
  // results - List of results in heap format. To get the top results,
  //           use the STL pop_heap() 
  void Search(const ImageDocument& doc, std::vector<SearchResult>* results,
              const DocSimilarityCalculator& similarityCalc=CosineSimilarity(),
              const ReweightDocSimilarity* reweighter=NULL) const;

private:
  typedef std::vector<boost::shared_ptr<ImageDocument> > DocCollection;
  typedef std::vector<ImageDocument*> DocList;

  // Collection of all the documents
  DocCollection docs_;

  // The inverse index. The index of the vector corresponds to the
  // TermID and then maps to a list of documents that contain that
  // term.
  std::vector<DocList> index_;

  // The inverse document frequency weights for all the terms squared
  std::vector<float> idf_;

  // True if all the documents added have been built into the index
  bool isAllBuilt_;

  // Helper functions

  // Adds a document to the inverse index by adding it to every entry
  // that its terms are associated with.
  void AddDocToInverseIndex(ImageDocument* doc);

  // Calculates the similarity between two documents using the cosine
  // similarity measure. Both documents must already be
  // multiplied by the tf-idf weighting.
  double CalcDocumentSimilarity(const ImageDocument& a,
                                const ImageDocument& b) const;

  // For serialization
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(docs_);
    ar & BOOST_SERIALIZATION_NVP(index_);
    ar & BOOST_SERIALIZATION_NVP(idf_);
    ar & BOOST_SERIALIZATION_NVP(isAllBuilt_);
  }


  // Evil constructors
  ImageSearchIndex(const ImageSearchIndex&);
  ImageSearchIndex& operator=(const ImageSearchIndex&);
};

}

#endif // _SPECIES_ID_IMAGE_SEARCH_INDEX_H__
