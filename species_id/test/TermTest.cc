#include "Term.h"

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;
using namespace boost::archive;
using namespace boost;

namespace species_id {

class TermTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

};

TEST_F(TermTest, valOnlyTerm) {
  Term term(1.0);

  EXPECT_FLOAT_EQ(1.0, term.val());
  EXPECT_EQ(NULL, term.coords());
}

TEST_F(TermTest, termWithCoord) {
  Term term(2.0, 3, 5);
  
  EXPECT_FLOAT_EQ(2.0, term.val());
  ASSERT_EQ(term.coords()->size(), 1u);
  const Term::Coord& c = (*term.coords())[0];
  EXPECT_FLOAT_EQ(3, c.first);
  EXPECT_FLOAT_EQ(5, c.second);
}

TEST_F(TermTest, termWithMultipleCoords) {
  Term term(2.0, 3, 5);
  term.AddTerm(3.2, 9, 1);
  
  EXPECT_FLOAT_EQ(5.2, term.val());
  ASSERT_EQ(term.coords()->size(), 2u);
  const Term::Coord& c1 = (*term.coords())[0];
  EXPECT_FLOAT_EQ(3, c1.first);
  EXPECT_FLOAT_EQ(5, c1.second);
  const Term::Coord& c2 = (*term.coords())[1];
  EXPECT_FLOAT_EQ(9, c2.first);
  EXPECT_FLOAT_EQ(1, c2.second);
}

TEST_F(TermTest, multTest) {
  Term term(4.0);
  
  ASSERT_FLOAT_EQ(4.0, term.val());
  term *= 2.0;
  ASSERT_FLOAT_EQ(8.0, term.val());
  term *= 0.25;
  ASSERT_FLOAT_EQ(2.0, term.val());
}

TEST_F(TermTest, addValOnlyTerm) {
  Term withCoord(5.0, 6, 1);

  withCoord.AddTerm(Term(3.0));
  EXPECT_FLOAT_EQ(8.0, withCoord.val());
  const Term::Coord& c1 = (*withCoord.coords())[0];
  EXPECT_FLOAT_EQ(6, c1.first);
  EXPECT_FLOAT_EQ(1, c1.second);

  Term noCoord(6.0);
  noCoord.AddTerm(Term(3.0));
  EXPECT_FLOAT_EQ(9.0, noCoord.val());
  EXPECT_EQ(NULL, noCoord.coords());
}

TEST_F(TermTest, addTermWithCoords) {
  Term withCoord(5.0, 6, 1);
  Term toAdd(1.0, 8, 2);

  withCoord.AddTerm(toAdd);
  EXPECT_FLOAT_EQ(6.0, withCoord.val());
  const Term::Coord& c1 = (*withCoord.coords())[0];
  EXPECT_FLOAT_EQ(6, c1.first);
  EXPECT_FLOAT_EQ(1, c1.second);
  const Term::Coord& c2 = (*withCoord.coords())[1];
  EXPECT_FLOAT_EQ(8, c2.first);
  EXPECT_FLOAT_EQ(2, c2.second);

  Term noCoord(6.0);
  noCoord.AddTerm(toAdd);
  EXPECT_FLOAT_EQ(7.0, noCoord.val());
  ASSERT_EQ(noCoord.coords()->size(), 1u);
  const Term::Coord& c3 = (*noCoord.coords())[0];
  EXPECT_FLOAT_EQ(8, c3.first);
  EXPECT_FLOAT_EQ(2, c3.second);
}

TEST_F(TermTest, addTermWithMultipleCoords) {
  Term withCoord(5.0, 6, 1);
  Term toAdd(1.0, 8, 2);
  toAdd.AddTerm(3.0, -1, 10);

  withCoord.AddTerm(toAdd);
  EXPECT_FLOAT_EQ(9.0, withCoord.val());
  const Term::Coord& c1 = (*withCoord.coords())[0];
  EXPECT_FLOAT_EQ(6, c1.first);
  EXPECT_FLOAT_EQ(1, c1.second);
  const Term::Coord& c2 = (*withCoord.coords())[1];
  EXPECT_FLOAT_EQ(8, c2.first);
  EXPECT_FLOAT_EQ(2, c2.second);
  const Term::Coord& c3 = (*withCoord.coords())[2];
  EXPECT_FLOAT_EQ(-1, c3.first);
  EXPECT_FLOAT_EQ(10, c3.second);
}

TEST_F(TermTest, addTermWithNanValue) {
  Term term(3.0);
  
  EXPECT_FLOAT_EQ(3.0, term.val());

  term.AddTerm(numeric_limits<float>::quiet_NaN());
  EXPECT_FLOAT_EQ(3.0, term.val());

  term.AddTerm(-numeric_limits<float>::quiet_NaN(), 4, 5);
  EXPECT_FLOAT_EQ(3.0, term.val());
  EXPECT_EQ(NULL, term.coords());
}

TEST_F(TermTest, addTermWithInfValue) {
  Term term(3.0);
  
  EXPECT_FLOAT_EQ(3.0, term.val());

  term.AddTerm(numeric_limits<float>::infinity());
  EXPECT_FLOAT_EQ(3.0, term.val());

  term.AddTerm(-numeric_limits<float>::infinity(), 4, 5);
  EXPECT_FLOAT_EQ(3.0, term.val());
  EXPECT_EQ(NULL, term.coords());
}

class ArchiveTest : public ::testing::Test {
protected:
  scoped_ptr<stringstream> buffer;

  virtual void SetUp() {
    buffer.reset(new stringstream(ios_base::out | ios_base::in));
  }

  void WriteToBuf(const Term& term) {
    xml_oarchive archive(*buffer);
    archive << BOOST_SERIALIZATION_NVP(term);
  }

  shared_ptr<Term> ReadFromBuf() {
    xml_iarchive archive(*buffer);
    shared_ptr<Term> retval(new Term());
    archive >> serialization::make_nvp("term", *retval);

    return retval;
  }

  // Checks that two terms are equal
  void ExpectEqTerm(const Term& a, const Term& b) {
    EXPECT_FLOAT_EQ(a.val(), b.val());
    
    EXPECT_EQ(a.coords() == NULL, b.coords() == NULL);
    if (a.coords() == NULL) {
      return;
    }
    EXPECT_TRUE(*a.coords() == *b.coords());
    ASSERT_EQ(a.coords()->size(), b.coords()->size());
    for (unsigned int i = 0; i < a.coords()->size(); i++) {
      const Term::Coord& aCoord = (*a.coords())[i];
      const Term::Coord& bCoord = (*b.coords())[i];
      EXPECT_EQ(aCoord.first, bCoord.first);
      EXPECT_EQ(aCoord.second, bCoord.second);
    }
  }

  // Does a round trip test
  void RoundTripTest(const Term& in) {
    Term::Ptr outExpected = in.copy();
    WriteToBuf(in);
    ExpectEqTerm(*outExpected, *(ReadFromBuf()));
  }

};

TEST_F(ArchiveTest, emptyTerm) {
  RoundTripTest(Term());
}

TEST_F(ArchiveTest, simpleTerm) {
  RoundTripTest(Term(5.0));
}

TEST_F(ArchiveTest, coordTerm) {
  RoundTripTest(Term(5.0, 3, 6));
}

TEST_F(ArchiveTest, multiCoordTerm) {
  Term term(5.0, 3, 6);
  term.AddTerm(10, 9, 12);
  RoundTripTest(term);
}

TEST_F(ArchiveTest, termWithNan) {
  Term term(numeric_limits<float>::quiet_NaN());
  RoundTripTest(term);
}

TEST_F(ArchiveTest, termWithInf) {
  Term term(numeric_limits<float>::infinity());
  RoundTripTest(term);
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
