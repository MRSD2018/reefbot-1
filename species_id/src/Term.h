// A "term" in an image that represents a weight in the document
// vector.
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: July 2010

#ifndef _SPECIES_ID_TERM_H__
#define _SPECIES_ID_TERM_H__

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <deque>
#include <limits>
#include <utility>

namespace species_id {

class Term {
public:
  typedef std::pair<float, float> Coord;
  typedef boost::shared_ptr<Term> Ptr;

  Term() : val_(0), coords_() {}
  
  Term(float val, float x=std::numeric_limits<float>::quiet_NaN(),
       float y=std::numeric_limits<float>::quiet_NaN()) : val_(0),
                                                          coords_(NULL)
    { AddTerm(val, x, y); }

  // Getters and setters
  float val() const { return val_; }
  const std::deque<Coord>* coords() const { return coords_.get(); }
  void SetVal(float val) { val_ = val; }

  // Adds a term to the running sum and appends the location if it's
  // specified.
  void AddTerm(float val, float x=std::numeric_limits<float>::quiet_NaN(),
               float y=std::numeric_limits<float>::quiet_NaN());
  // Adds all the elements in the supplied term to this one
  void AddTerm(const Term& term);

  // Multiplies the value in the term by a constant
  Term& operator*=(float v) { val_ *= v; return *this; }

  bool operator==(const Term& t) const {
    return t.val_ == val_ && coords_.get() && t.coords_.get() &&
      *coords_ == *t.coords_;
  }
  bool operator!=(const Term& t) const {
    return !(*this == t);
  }

  // Perform a deep copy of the object
  Ptr copy() const;

private:  
  // The weight of the term
  float val_;

  // List of coordinates where this term was found in the image (can be NULL)
  boost::scoped_ptr<std::deque<Coord> > coords_;

  // Routines to pickle this object.
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(val_);
    ar & BOOST_SERIALIZATION_NVP(coords_);
  }

  // Evil constructors
  Term(const Term& t);
  Term& operator=(const Term& t);
};

} // namespace


#endif // _SPECIES_ID_TERM_H__
