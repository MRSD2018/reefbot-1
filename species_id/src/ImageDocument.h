// Describes an image using a bag of words search framework. The image
// is effectively a sparse array of "term" counts with optional sets
// of (x,y) coordinates in the image associated with that term.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: July 2010

#ifndef _SPECIES_ID_IMAGE_DOCUMENT_H__
#define _SPECIES_ID_IMAGE_DOCUMENT_H__

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/scoped_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <math.h>
#include <stdint.h>
#include <vector>

#include "base/BasicTypes.h"
#include "base/StringPiece.h"
#include "Term.h"
#include "boost_serialize_unordered_map.hpp"

namespace species_id {

typedef uint32 TermID;

class ImageDocument {
public:

  typedef boost::unordered_map<TermID, Term::Ptr> TermMap;

  ImageDocument() : terms_(), isNormalized_(false),
                    length_(LENGTH_NOT_CALCULATED),
                    label_(""), metaData_()  {}
  ImageDocument(const StringPiece& label) : terms_(), isNormalized_(false),
                                            length_(LENGTH_NOT_CALCULATED),
                                            label_(), metaData_() {
    label.CopyToString(&label_);
  }
  
  
  // Add a single term to the document with optional (x,y)
  // location. If the same term is already in the document, the value
  // is added to the existing sum and the (x,y) is added to the list.
  //
  // Inputs:
  // id - ID of the term to add
  // val - weight of the term to add to the vector
  // (x,y) - optional coordinates in the image where the term was found
  void AddTerm(TermID id, float val=1.0, float x=NAN, float y=NAN) {
    AddTermImpl(id, NULL, val, x, y);
  }
  void AddTerm(TermID id, const Term& term) {
    AddTermImpl(id, &term);
  }

  // Sets the value of a term
  void SetTerm(TermID id, float val);

  // Adds all the terms from another document with an optional offset
  // to the term indicies
  //
  // Inputs:
  // doc - Document to grab the terms from
  // offset - Index offset when putting into this doc
  // minVal - Minimum value of the term in order to add it to the doc
  void AddTermsFromDoc(const ImageDocument& doc, TermID offset=0,
                       float minVal=0);

  // Returns the number of unique terms in the image
  int nUniqueTerms() const { return terms_.size(); }

  // Normalizes all the terms so that they sum to 1. Note that this
  // effectivly multiplies the document by the term frequency vector.
  void Normalize();

  // Term by term multiplication and stores the result in this vector
  ImageDocument& operator*=(const std::vector<float>& v);
  ImageDocument& operator*=(double v);

  // Getters
  bool IsNormalized() const { return isNormalized_; }
  float length() const;
  const std::string& label() const { return label_; }
  void SetLabel(const std::string& label) { label_ = label; }

  const std::string& metaData() const {
    return metaData_.get() == NULL ? EMPTY_STRING : *metaData_;
  }
  void SetMetaData(const std::string& metaData) {
    metaData_.reset(new std::string(metaData));
  }

  // Returns the value associated with a given id
  float GetVal(TermID id) const;

  // Returns the term associated with TermID or NULL if it doesn't exist
  Term* GetTerm(TermID id) const;

  // Returns the list of coordinates associated with a given
  // id. Returns NULL if there are no coordinates.
  const std::deque<Term::Coord>* GetCoords(TermID id) const;

  // Iterators through the non-zero valued terms
  TermMap::const_iterator begin() const {
    return terms_.begin();
  }
  TermMap::const_iterator end() const {
    return terms_.end();
  }

  // Creates a deep copy of the image document
  boost::shared_ptr<ImageDocument> copy() const;

  // Removes all the terms whose values are below a threshold
  void RemoveTermsBelowThresh(float thresh);

private:
  // Map from ID -> term
  TermMap terms_;

  // Have the term values been normalized
  bool isNormalized_;

  // The euclidean length of the term vector. If it's negative, then
  // it hasn't been calculated.
  static const double LENGTH_NOT_CALCULATED = -1;
  mutable float length_;

  // String that specifies an arbitrary label for this document
  std::string label_;

  // Application specific meta data
  boost::shared_ptr<std::string> metaData_;
  static const std::string EMPTY_STRING;

  // Implementation that only does the necessary copies.
  void AddTermImpl(TermID id, const Term* termPtr,
                   float val=1.0, float x=NAN, float y=NAN);

  // Routines to pickle this object.
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive& ar, const unsigned int version) const {
    ar << BOOST_SERIALIZATION_NVP(terms_);
    ar << BOOST_SERIALIZATION_NVP(isNormalized_);
    ar << BOOST_SERIALIZATION_NVP(label_);
    if (version > 1) {
      if (metaData_.get()) {
        ar << boost::serialization::make_nvp("meta_data", *metaData_);
      } else {
        ar << boost::serialization::make_nvp("meta_data", EMPTY_STRING);
      }
    }
  }

  template<class Archive>
  void load(Archive& ar, const unsigned int version) {
    ar >> BOOST_SERIALIZATION_NVP(terms_);
    ar >> BOOST_SERIALIZATION_NVP(isNormalized_);
    ar >> BOOST_SERIALIZATION_NVP(label_);
    if (version > 1) {
      metaData_.reset(new string(""));
      ar >> boost::serialization::make_nvp("meta_data", *metaData_);
    }
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  // Evil constructors
  DISALLOW_EVIL_CONSTRUCTORS(ImageDocument);
};

}
BOOST_CLASS_VERSION(species_id::ImageDocument, 2);


#endif // _SPECIES_ID_IMAGE_DOCUMENT_H__
