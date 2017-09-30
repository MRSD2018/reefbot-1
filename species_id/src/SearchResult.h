// Specifies a document returned as a result of a search
//
// Author: Mark Desnoyer markd@cmu.edu 
// Date: July 2010

#ifndef _SPECIES_ID_SEARCH_RESULT_H__
#define _SPECIES_ID_SEARCH_RESULT_H__

#include "ImageDocument.h"

namespace species_id {

class SearchResult {
public:
  SearchResult(float score, const ImageDocument* doc) 
    : score_(score), doc_(doc) {}

  float score() const { return score_; }
  const ImageDocument* doc() const { return doc_; }

  bool operator<(const SearchResult& other) const {
    return score_ < other.score_;
  }

private:
  float score_;
  const ImageDocument* doc_;
};

}

bool operator< (const species_id::SearchResult& a,
                const species_id::SearchResult& b);

#endif // _SPECIES_ID_SEARCH_RESULT_H__
