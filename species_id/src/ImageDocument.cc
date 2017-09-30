// Describes an image using a bag of words search framework. The image
// is effectively a sparse array of "term" counts with optional sets
// of (x,y) coordinates in the image associated with that term.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: July 2010

#include "ImageDocument.h"

#include <math.h>
#include <ros/ros.h>
#include <limits>
#include <boost/math/special_functions/fpclassify.hpp>

using namespace std;
using boost::shared_ptr;

namespace species_id {

const string ImageDocument::EMPTY_STRING("");

// Add a single term to the document with optional (x,y)
// location. If the same term is already in the document, the value
// is added to the existing sum and the (x,y) is added to the list.
//
// Inputs:
// id - ID of the term to add
// val - weight of the term to add to the vector
// (x,y) - optional coordinates in the image where the term was found
void ImageDocument::AddTermImpl(TermID id, const Term* termPtr,
                                float val, float x, float y) {
  if ((termPtr && (fabs(termPtr->val()) < 10*numeric_limits<float>::epsilon() &&
                   (termPtr->coords() == NULL or termPtr->coords()->size() == 0))) ||
      (!termPtr && val < 10*numeric_limits<float>::epsilon() &&
       !boost::math::isfinite(x) && !boost::math::isfinite(y))) {
    // No point in adding a zero term
    return;
  }

  isNormalized_ = false;
  length_ = LENGTH_NOT_CALCULATED;

  TermMap::iterator loc = terms_.find(id);
  if (loc == terms_.end()) {
    // Term is not in the document, so add it
    if (termPtr) {
      terms_[id] = termPtr->copy();
    } else {
      terms_[id] = Term::Ptr(new Term(val, x, y));
    }
  } else {
    // Term is already in the document, so add to it
    if (termPtr) {
      loc->second->AddTerm(*termPtr);
    } else {
      loc->second->AddTerm(val, x, y);
    }
  }    
}

void ImageDocument::SetTerm(TermID id, float val) {
  bool isZero = fabs(val) < 10*numeric_limits<float>::epsilon();

  isNormalized_ = false;
  length_ = LENGTH_NOT_CALCULATED;

  TermMap::iterator loc = terms_.find(id);
  if (loc == terms_.end()) {
    // Term is not in the document, so add it
    if (!isZero) {
      terms_[id] = shared_ptr<Term>(new Term(val));
    }
  } else {
    // Term is already in the document, so set it
    loc->second->SetVal(val);
  }
}

void ImageDocument::AddTermsFromDoc(const ImageDocument& doc,
                                    TermID offset,
                                    float minVal) {
  for (TermMap::const_iterator termI = doc.begin();
       termI != doc.end();
       ++termI) {
    if (termI->second->val() > minVal) {
      AddTerm(termI->first + offset, *termI->second);
    }
  }
}

// Normalizes the terms so that they sum to 1
void ImageDocument::Normalize() {
  if (isNormalized_) {
    return;
  }

  double sum = 0.0;

  // Figure out the total sum of the terms
  for (TermMap::const_iterator i = begin();
       i != end();
       ++i) {
    sum += i->second->val();
  }

  if (fabs(sum) < numeric_limits<float>::epsilon()*10) {
    // Sum is zero, so we'll get a nan. Leave things along
    isNormalized_ = true;
    length_ = LENGTH_NOT_CALCULATED;
    return;
  }

  // Now update the vector
  double factor = 1. / sum;
  (*this) *= factor;

  isNormalized_ = true;
  length_ = LENGTH_NOT_CALCULATED;
}

float ImageDocument::length() const {
  if (length_ < 0) {
    double sum = 0.0;

    // Figure out the term by term sum
    for (TermMap::const_iterator i = begin();
         i != end();
         ++i) {
      sum += (i->second->val() * i->second->val()) ;
    }
    length_ = sqrt(sum);
  }

  return length_;
}

// Term by term multiplication
ImageDocument& ImageDocument::operator*=(const vector<float>& v) {
  isNormalized_ = false;
  length_ = LENGTH_NOT_CALCULATED;

  // Now update the vector
  for (TermMap::iterator i = terms_.begin();
       i != terms_.end();
       ++i) {
    TermID id = i->first;
    double val = 0.0;
    if (id < v.size()) {
      val = v[id];
    } else {
      ROS_WARN_STREAM_ONCE(
        "Vector to multiply by is missing an entry for id: " << id);
    }
    *(i->second) *= val;
  }
}

ImageDocument& ImageDocument::operator*=(double v) {
  isNormalized_ = false;
  length_ = LENGTH_NOT_CALCULATED;

  for (TermMap::iterator i = terms_.begin();
       i != terms_.end();
       ++i) {
    *(i->second) *= v;
  }
}

float ImageDocument::GetVal(TermID id) const {
  Term* term = GetTerm(id);
  if (term == NULL) {
    return 0.0;
  }
  return term->val();
}

const std::deque<Term::Coord>* ImageDocument::GetCoords(TermID id) const {
  Term* term = GetTerm(id);
  if (term == NULL || term->coords() == NULL) {
    return NULL;
  }
  return term->coords();
}

shared_ptr<ImageDocument> ImageDocument::copy() const {
  shared_ptr<ImageDocument> copy(new ImageDocument());

  copy->isNormalized_ = isNormalized_;

  for (TermMap::const_iterator i = begin();
       i != end();
       ++i) {
    copy->terms_[i->first] = i->second->copy();
  }

  copy->label_ = label_;
  copy->metaData_ = metaData_;

  return copy;
}

Term* ImageDocument::GetTerm(TermID id) const {
  TermMap::const_iterator loc = terms_.find(id);
  if (loc == terms_.end()) {
    return NULL;
  }
  return loc->second.get();
}

void ImageDocument::RemoveTermsBelowThresh(float thresh) {
  for(TermMap::iterator i = terms_.begin();
      i != terms_.end();) {
    if (i->second->val() < thresh) {
      i = terms_.erase(i);
    } else {
      ++i;
    }
  }
}

}
