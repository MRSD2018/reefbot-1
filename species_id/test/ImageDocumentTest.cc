#include "ImageDocument.h"

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
using namespace boost::serialization;

namespace species_id {

class ImageDocumentTest : public ::testing::Test {
public:
  ImageDocumentTest() : doc() {}

protected:
  ImageDocument doc;
  
  // Compared a set of coordinates to a list of (x,y) coordinates
  virtual void ExpectEqCoords(const deque<Term::Coord>* actual,
                              const float expected[][2]) {
    ASSERT_TRUE(actual != NULL);

    for (unsigned int i=0; i < actual->size(); i++) {
      EXPECT_FLOAT_EQ(expected[i][0], (*actual)[i].first);
      EXPECT_FLOAT_EQ(expected[i][1], (*actual)[i].second);
    }
  }

};

TEST_F(ImageDocumentTest, emptyTest) {
  EXPECT_EQ(doc.nUniqueTerms(), 0);
  EXPECT_FALSE(doc.IsNormalized());
  EXPECT_FALSE(doc.GetCoords(0));
  EXPECT_FLOAT_EQ(doc.GetVal(0), 0.0);
  EXPECT_FLOAT_EQ(doc.length(), 0.0);
  EXPECT_EQ(doc.label(), "");
  EXPECT_EQ(doc.metaData(), "");
}

TEST_F(ImageDocumentTest, addZeroTerm) {
  // Adding a zero term keeps the value at zero
  doc.AddTerm(53, 0.0);
  EXPECT_FLOAT_EQ(doc.GetVal(53), 0.0);
  EXPECT_TRUE(doc.GetCoords(53) == NULL);
  EXPECT_FLOAT_EQ(doc.length(), 0.0);
}

TEST_F(ImageDocumentTest, addZeroTermWithCoords) {
  // Adding a zero term keeps the value at zero and records the coordinates
  doc.AddTerm(53, 0.0, 6, 7);
  EXPECT_FLOAT_EQ(doc.GetVal(53), 0.0);
  float c1[][2] = {{6, 7}};
  ExpectEqCoords(doc.GetCoords(53), c1);
  EXPECT_FLOAT_EQ(doc.length(), 0.0);
}

TEST_F(ImageDocumentTest, addUniqueTerms) {
  doc.AddTerm(53, 1.5, 6, 7);
  doc.AddTerm(51, 0.5, 9, 8);

  EXPECT_EQ(doc.nUniqueTerms(), 2);
  EXPECT_FALSE(doc.IsNormalized());

  EXPECT_FLOAT_EQ(doc.GetVal(53), 1.5);
  float c1[][2] = {{6, 7}};
  ExpectEqCoords(doc.GetCoords(53), c1);
  EXPECT_FLOAT_EQ(doc.GetVal(51), 0.5);

  float c2[][2] = {{9, 8}};
  ExpectEqCoords(doc.GetCoords(51), c2);
}

TEST_F(ImageDocumentTest, incrementTerm) {
  doc.AddTerm(53, 1.5, 6, 7);
  doc.AddTerm(53, 0.5, 9, 8);

  EXPECT_EQ(doc.nUniqueTerms(), 1);
  EXPECT_FALSE(doc.IsNormalized());

  EXPECT_FLOAT_EQ(doc.GetVal(53), 2.0);
  float c1[][2] = { {6, 7}, {9, 8} };
  ExpectEqCoords(doc.GetCoords(53), c1);
}

TEST_F(ImageDocumentTest, incrementTermNoCoords) {
  doc.AddTerm(53, 1.5);
  doc.AddTerm(49, 5.3);
  doc.AddTerm(53, 0.5);

  EXPECT_EQ(doc.nUniqueTerms(), 2);
  EXPECT_FALSE(doc.IsNormalized());

  EXPECT_FLOAT_EQ(doc.GetVal(53), 2.0);
  EXPECT_TRUE(doc.GetCoords(53) == NULL);

  EXPECT_FLOAT_EQ(doc.GetVal(49), 5.3);
  EXPECT_TRUE(doc.GetCoords(49) == NULL);
}

TEST_F(ImageDocumentTest, testNormalization) {
  // Tests that the normalization works and is flagged, then, if a new
  // term is added, the structure is no longer flagged as normalized.
  doc.AddTerm(53, 1.0);
  doc.AddTerm(49, 2.0);
  doc.AddTerm(55, 0.5);
  doc.AddTerm(1, 1.5);

  ASSERT_EQ(doc.nUniqueTerms(), 4);
  EXPECT_FALSE(doc.IsNormalized());

  EXPECT_FLOAT_EQ(doc.GetVal(53), 1.0);
  EXPECT_FLOAT_EQ(doc.GetVal(49), 2.0);
  EXPECT_FLOAT_EQ(doc.GetVal(55), 0.5);
  EXPECT_FLOAT_EQ(doc.GetVal(1), 1.5);

  doc.Normalize();
  EXPECT_TRUE(doc.IsNormalized());
  EXPECT_FLOAT_EQ(doc.GetVal(53), 0.2);
  EXPECT_FLOAT_EQ(doc.GetVal(49), 0.4);
  EXPECT_FLOAT_EQ(doc.GetVal(55), 0.1);
  EXPECT_FLOAT_EQ(doc.GetVal(1), 0.3);

  doc.AddTerm(3, 6);
  EXPECT_FALSE(doc.IsNormalized());
  EXPECT_FLOAT_EQ(doc.GetVal(3), 6);
}

TEST_F(ImageDocumentTest, testZeroNormalization) {
  // Tests when zero terms are added and make sure that the terms are
  // still zero after normalization.
  doc.AddTerm(53, 0.0);
  EXPECT_FLOAT_EQ(doc.GetVal(53), 0.0);
  EXPECT_FALSE(doc.IsNormalized());

  doc.Normalize();
  EXPECT_TRUE(doc.IsNormalized());
  EXPECT_FLOAT_EQ(doc.GetVal(53), 0.0);
}

TEST_F(ImageDocumentTest, testZeroNormalizationWithCoords) {
  // Tests when zero terms are added and make sure that the terms are
  // still zero after normalization.
  doc.AddTerm(53, 0.0, 6, 7);
  EXPECT_FLOAT_EQ(doc.GetVal(53), 0.0);
  EXPECT_FALSE(doc.IsNormalized());

  doc.Normalize();
  EXPECT_TRUE(doc.IsNormalized());
  EXPECT_FLOAT_EQ(doc.GetVal(53), 0.0);
}

TEST_F(ImageDocumentTest, testMultVec) {
  // Test the term by term multiplication by a floating point vector
  vector<float> vec;
  vec.push_back(0.0);
  vec.push_back(1.0);
  vec.push_back(0.2);
  vec.push_back(5.0);
  vec.push_back(-0.5);

  doc.AddTerm(2, 5.0);
  doc.AddTerm(4, 6.0);
  doc.AddTerm(0, 100);
  doc *= vec;

  EXPECT_FLOAT_EQ(doc.GetVal(0), 0);
  EXPECT_FLOAT_EQ(doc.GetVal(1), 0);
  EXPECT_FLOAT_EQ(doc.GetVal(2), 1.0);
  EXPECT_FLOAT_EQ(doc.GetVal(3), 0);
  EXPECT_FLOAT_EQ(doc.GetVal(4), -3.0);
}

TEST_F(ImageDocumentTest, testMultScalar) {
  // Test the scalar multiplication
  doc.AddTerm(2, 5.0);
  doc.AddTerm(4, 6.0);
  doc.AddTerm(0, 100);
  doc *= 2.5;

  EXPECT_FLOAT_EQ(doc.GetVal(0), 250);
  EXPECT_FLOAT_EQ(doc.GetVal(1), 0);
  EXPECT_FLOAT_EQ(doc.GetVal(2), 12.5);
  EXPECT_FLOAT_EQ(doc.GetVal(3), 0);
  EXPECT_FLOAT_EQ(doc.GetVal(4), 15.0);
}


// Makes sure that the copy is a true deep copy
TEST_F(ImageDocumentTest, testCopy) {
  doc.AddTerm(5, 0.5, 6, 3);
  doc.AddTerm(3, 1.0);
  doc.AddTerm(9, 2.5, 4, 7);
  doc.AddTerm(5, 1.0, 9, 1);
  doc.SetLabel("ALabel");
  doc.SetMetaData("Meta Data");
  boost::shared_ptr<ImageDocument> copy(doc.copy());

  EXPECT_EQ(copy->label(), doc.label());
  EXPECT_EQ(copy->metaData(), doc.metaData());

  doc.Normalize();
  
  EXPECT_TRUE(doc.IsNormalized());
  EXPECT_FALSE(copy->IsNormalized());
  EXPECT_FLOAT_EQ(copy->GetVal(5), 1.5);
  EXPECT_FLOAT_EQ(doc.GetVal(5), 0.3);
  EXPECT_FLOAT_EQ(copy->GetVal(3), 1.0);
  EXPECT_FLOAT_EQ(doc.GetVal(3), 0.2);
  float c1[][2] = { {6, 3}, {9, 1} };
  ExpectEqCoords(doc.GetCoords(5), c1);
  ExpectEqCoords(copy->GetCoords(5), c1);

  // Now add another coordinate
  doc.AddTerm(5, 3.0, 7, 8);
  float c2[][2] = { {6, 3}, {9, 1}, {7, 8} };
  ExpectEqCoords(doc.GetCoords(5), c2);
  ExpectEqCoords(copy->GetCoords(5), c1);

  // Now change the label and meta data
  doc.SetLabel("BLabel");
  doc.SetMetaData("NewMeta");
  EXPECT_EQ(copy->label(), "ALabel");
  EXPECT_EQ(copy->metaData(), "Meta Data");
  EXPECT_NE(copy->label(), doc.label());
  EXPECT_NE(copy->metaData(), doc.metaData());
}

TEST_F(ImageDocumentTest, labelCopy) {
  ImageDocument docA("A Doc");

  EXPECT_EQ(docA.label(), "A Doc");

  boost::shared_ptr<ImageDocument> copy(docA.copy());
  EXPECT_EQ(copy->label(), docA.label());

}

TEST_F(ImageDocumentTest, testLength) {
  doc.AddTerm(0, 3);
  EXPECT_FLOAT_EQ(doc.length(), 3);

  // Make sure length is updated when a new term is added
  doc.AddTerm(1, 4);
  EXPECT_FLOAT_EQ(doc.length(), 5);

  // Make sure that length is updated when the vector is normalized
  doc.Normalize();
  EXPECT_FLOAT_EQ(doc.length(), sqrt(25./49.));

  // Make sure that length is updated when the vector is multiplied by some terms
  vector<float> vec;
  vec.push_back(35./3);
  vec.push_back(21);

  doc *= vec;
  EXPECT_FLOAT_EQ(doc.length(), 13.0);
}


class ArchiveTest : public ::testing::Test {
public:
  ArchiveTest() : doc(), buffer() {}

protected:
  scoped_ptr<ImageDocument> doc;
  scoped_ptr<stringstream> buffer;

  virtual void SetUp() {
    buffer.reset(new stringstream(ios_base::out | ios_base::in));
    doc.reset(new ImageDocument());
  }

  void WriteToBuf(const ImageDocument& term) {
    xml_oarchive archive(*buffer);
    archive << BOOST_SERIALIZATION_NVP(term);
  }

  shared_ptr<ImageDocument> ReadFromBuf() {
    xml_iarchive archive(*buffer);
    shared_ptr<ImageDocument> retval(new ImageDocument());
    archive >> serialization::make_nvp("term", *retval);

    return retval;
  }

  // Checks that two documents are equal
  void ExpectEqDoc(const ImageDocument& a, const ImageDocument& b) {
    EXPECT_EQ(a.IsNormalized(), b.IsNormalized());
    EXPECT_EQ(a.label(), b.label());
    EXPECT_EQ(a.metaData(), b.metaData());
    ASSERT_EQ(a.nUniqueTerms(), b.nUniqueTerms());

    for (ImageDocument::TermMap::const_iterator i = a.begin();
         i != a.end(); ++i) {
      EXPECT_EQ(i->second->val(), b.GetVal(i->first));
      const deque<Term::Coord>* aCoords(i->second->coords());
      const deque<Term::Coord>* bCoords = b.GetCoords(i->first);
      ASSERT_EQ(aCoords == NULL, bCoords == NULL);
      if (bCoords != NULL) {
        EXPECT_TRUE(*aCoords == *bCoords);
      }
    }
  }

  // Does a round trip test
  void RoundTripTest(const ImageDocument& in) {
    shared_ptr<ImageDocument> outExpected = in.copy();
    WriteToBuf(in);
    ExpectEqDoc(*outExpected, *(ReadFromBuf()));
  }

};

TEST_F(ArchiveTest, emptyDoc) {
  RoundTripTest(*doc);
}

TEST_F(ArchiveTest, singleTerm) {
  doc->AddTerm(5, 3.0);
  RoundTripTest(*doc);
}

TEST_F(ArchiveTest, multipleValOnlyTerms) {
  doc->AddTerm(5, 3.0);
  doc->AddTerm(7, 1.0);
  doc->AddTerm(62, -1.0);
  doc->AddTerm(5, -1.0);
  RoundTripTest(*doc);
}

TEST_F(ArchiveTest, singleCoordTerm) {
  doc->AddTerm(5, 3.0, 6, 3);
  RoundTripTest(*doc);
}

TEST_F(ArchiveTest, multipleCoordTerms) {
  doc->AddTerm(5, 3.0, 6, 3);
  doc->AddTerm(3, 1.0);
  doc->AddTerm(9, 2.2, 4, 7);
  doc->AddTerm(5, 1.0, 9, 1);
  RoundTripTest(*doc);
}

TEST_F(ArchiveTest, normalizedTest) {
  doc->AddTerm(5, 3.0, 6, 3);
  doc->AddTerm(3, 1.0);
  doc->AddTerm(9, 2.2, 4, 7);
  doc->AddTerm(5, 1.0, 9, 1);
  boost::shared_ptr<ImageDocument> unNormalized(doc->copy());
  RoundTripTest(*doc);
  doc->Normalize();
  
  RoundTripTest(*doc);
}

TEST_F(ArchiveTest, labeledDoc) {
  doc.reset(new ImageDocument("My Document"));
  doc->AddTerm(4);

  RoundTripTest(*doc);
}


TEST_F(ArchiveTest, metaDataDoc) {
  doc.reset(new ImageDocument("My Document"));
  doc->AddTerm(4);
  doc->AddTerm(9, 2.2, 4, 7);
  doc->AddTerm(3, 1.0);
  doc->SetMetaData("Meta data now");
  EXPECT_EQ(doc->metaData(), "Meta data now");

  RoundTripTest(*doc);
} 

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
