#include "Term.h"
#include <boost/math/special_functions/fpclassify.hpp>

using namespace std;

namespace species_id {

void Term::AddTerm(float val, float x, float y) {
  if (!boost::math::isfinite(val)) {
    return;
  }

  if (boost::math::isfinite(x) && boost::math::isfinite(y)) {
    if (coords_.get() == NULL) {
      coords_.reset(new std::deque<Coord>());
    }
    coords_->push_back(Coord(x, y));
  }
  val_ += val;
}

void Term::AddTerm(const Term& term) {
  // Add the value
  val_ += term.val();

  // Add all the coordinates
  if (term.coords() != NULL) {
    if (coords_.get() == NULL) {
      coords_.reset(new std::deque<Coord>());
    }
    coords_->insert(coords_->end(), term.coords()->begin(),
                    term.coords()->end());
  }
}

Term::Ptr Term::copy() const {
  Term::Ptr term(new Term());

  term->val_ = val_;
  if (coords_.get() != NULL) {
    term->coords_.reset(new std::deque<Coord>(*coords_));
  }

  return term;
}

}
