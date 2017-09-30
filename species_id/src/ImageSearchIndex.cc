// An image search index. Each document is specifiec as a list of
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

#include "ImageSearchIndex.h"

#include <algorithm>
#include <ext/hash_set>
#include <math.h>
#include <ros/ros.h>

using namespace boost;
using namespace std;

namespace std { using namespace __gnu_cxx; }

namespace  __gnu_cxx {
template<> struct hash<const species_id::ImageDocument*> {
  size_t operator()(const species_id::ImageDocument* const c) const {
    return reinterpret_cast<size_t>(c);
  }
};
}

namespace species_id {

ImageSearchIndex::ImageSearchIndex()
  : docs_(), index_(), idf_(), isAllBuilt_(false) {}

ImageDocument* ImageSearchIndex::AddDocument(const ImageDocument& doc) {
  return AddDocument(doc.copy());
}

ImageDocument* ImageSearchIndex::AddDocument(shared_ptr<ImageDocument> doc) {
  isAllBuilt_ = false;

  docs_.push_back(doc);
  return doc.get();
}

// Functor to compare two documents by the value of a term. Causes it
// to be a descending order.
struct DocComparator {
  TermID id;

  DocComparator(TermID id_) : id(id_) {}
  bool operator()(const ImageDocument* docA, const ImageDocument* docB) {
    ROS_ASSERT(docA != NULL && docB != NULL);
    return docA->GetVal(id) > docB->GetVal(id);
  }
};

void ImageSearchIndex::BuildIndex() {
  ROS_INFO_STREAM("Building index with " << docs_.size() << " documents.");

  ROS_INFO("Building the inverse index");
  for (DocCollection::iterator i = docs_.begin();
       i != docs_.end();
       ++i) {
    (**i).Normalize();
    AddDocToInverseIndex(i->get());
  }

  ROS_INFO("Calculating the IDF vector");
  if (idf_.size() != index_.size()) {
    ROS_FATAL("The index and the idf vectors are not the same size.");
    exit(4);
  }
  double idf;
  for (TermID i = 0; i < idf_.size(); i++) {
    // IDF is the log of the number of documents in the corpus divided
    // by the number with this term.
    idf = log((double)docs_.size()) - log((double)(index_[i].size()));
    if (isinf(idf) || isnan(idf)) {
      idf_[i] = 0;
    } else {
      idf_[i] = idf;
    }
  }

  ROS_INFO("Multiplying each document by the idf vector");
  for (DocCollection::iterator i = docs_.begin();
       i != docs_.end();
       ++i) {
    (**i) *= idf_;
  }

  ROS_INFO("Sorting the entries in the inverse index.");
  for (TermID i = 0; i < index_.size(); i++) {
    if (index_[i].size() > 1) {
      sort(index_[i].begin(), index_[i].end(), DocComparator(i));
    }
  }

  ROS_INFO("The index is built and can now be searched");
  isAllBuilt_ = true;
}

void ImageSearchIndex::Search(const ImageDocument& doc,
                              vector<SearchResult>* results,
                              const DocSimilarityCalculator& similarityCalc,
                              const ReweightDocSimilarity* reweighter) const {
  ROS_ASSERT(results != NULL);

  if (isAllBuilt_ == false) {
    ROS_WARN("Not all the documents have been built into the index");
  }

  if (results->size() > 0) {
    ROS_WARN("Results container already contained some results. "
             "They are being deleted");
    results->clear();
  }

  // See if we need to normalize the document (multiply by the tf term)
  shared_ptr<ImageDocument> docCopy = doc.copy();
  docCopy->Normalize();

  // Now multiply by the idf term
  (*docCopy) *= idf_;

  // Create the set of documents to look at based on having the same
  // terms as doc

  // TODO(mdesnoyer) optimize by scoring the terms with the highest
  // value in doc first.
  hash_set<const ImageDocument*> docsToScore;
  for (ImageDocument::TermMap::const_iterator termI = docCopy->begin();
       termI != docCopy->end();
       ++termI) {
    if (termI->first >= index_.size()) {
      ROS_WARN_STREAM_ONCE("Term id " << termI->first
                           << " was not in the index.");
      continue;
    }
    const DocList& docList = index_[termI->first];
    for (DocList::const_iterator docI = docList.begin();
         docI != docList.end();
         ++docI) {
      docsToScore.insert(*docI);
    }
  }

  vector<SearchResult> tempResults;
  vector<SearchResult>* initialResults = &tempResults;
  if (reweighter == NULL) {
    initialResults = results;
  }

  // Score the documents and keep them sorted using a heap
  for (hash_set<const ImageDocument*>::const_iterator docI =
         docsToScore.begin(); docI != docsToScore.end(); ++docI) {
    initialResults->push_back(
      SearchResult(similarityCalc.CalcDocumentSimilarity(**docI, *docCopy),
                   *docI));
    push_heap(initialResults->begin(), initialResults->end());
  }

  // Now do the reweighting if it was called for
  if (reweighter != NULL) {
    ROS_INFO_STREAM("Trying to reweight top 20 results of "
                    << docsToScore.size());

    // Reweight the first n results
    for (int i = 0; i < 20 && i < docsToScore.size(); i++) {
      pop_heap(initialResults->begin(), initialResults->end());
      const SearchResult& curResult = initialResults->back();
      results->push_back(
        SearchResult(reweighter->CalcNewSimilarity(*curResult.doc(),
                                                   *docCopy,
                                                   curResult.score()),
                     curResult.doc()));
      push_heap(results->begin(), results->end());
      initialResults->pop_back();

    }

    // Now push in any of the other results without reweighting them
    while(!initialResults->empty()) {
      pop_heap(initialResults->begin(), initialResults->end());
      SearchResult curResult = initialResults->back();
      results->push_back(curResult);
      push_heap(results->begin(), results->end());
      initialResults->pop_back();
    }
  }
}

void ImageSearchIndex::AddDocToInverseIndex(ImageDocument* doc) {
  ROS_ASSERT(doc != NULL);

  for (ImageDocument::TermMap::const_iterator termI = doc->begin();
       termI != doc->end();
       ++termI) {
    while (termI->first >= index_.size()) {
      index_.push_back(DocList());
      idf_.push_back(0.0);
    }
    index_[termI->first].push_back(doc);
  }
}

// This calculates the cosine similarity measure which is the dot
// product of the tf-idf values for each document divided by both
// document's lengths.
double ImageSearchIndex::CalcDocumentSimilarity(const ImageDocument& a,
                                                const ImageDocument& b) const {
  double score = 0;

  // Start by calculating the dot product by iterating through the smaller doc
  const ImageDocument* iterDoc = &a;
  const ImageDocument* lookupDoc = &b;
  if (a.nUniqueTerms() > b.nUniqueTerms()) {
    iterDoc = &b;
    lookupDoc = &a;
  }
  for(ImageDocument::TermMap::const_iterator termI = iterDoc->begin();
      termI != iterDoc->end(); ++termI) {
    score += (termI->second->val() * lookupDoc->GetVal(termI->first));
  }

  // Now divide by the lengths
  score /= a.length();
  score /= b.length();

  return score;
}


}
