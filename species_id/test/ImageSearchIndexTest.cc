#include "ImageSearchIndex.h"

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/random.hpp>
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

class ImageSearchIndexTest : public ::testing::Test {
public:
  ImageSearchIndexTest() {}

protected:
  scoped_ptr<ImageSearchIndex> index;
  vector<SearchResult> results;

  void SetUp() {
    index.reset(new ImageSearchIndex());
    results.clear();
  }

  // Helper function that adds a simple doc with a maximum of 5
  // terms. Set the term value to < 0 so that the term isn't added to
  // that doc.
  
  void AddSimpleDoc(float term0, float term1, float term2, float term3,
                    float term4, const string& label) {
    index->AddDocument(CreateSimpleDoc(term0, term1, term2, term3, term4,
                                      label));
  }

  shared_ptr<ImageDocument> CreateSimpleDoc(float term0, float term1,
                                            float term2, float term3,
                                            float term4,
                                            string label=string("")) {
    shared_ptr<ImageDocument> doc (new ImageDocument(label));
    MaybeAddTerm(doc.get(), 0, term0);
    MaybeAddTerm(doc.get(), 1, term1);
    MaybeAddTerm(doc.get(), 2, term2);
    MaybeAddTerm(doc.get(), 3, term3);
    MaybeAddTerm(doc.get(), 4, term4);

    return doc;
  }

  void MaybeAddTerm(ImageDocument* doc, TermID id, float term) {
    if (term > 0) {
      doc->AddTerm(id, term);
    }
  }

  // Does a search with a simple doc and fills results
  void SearchWithDoc(float term0, float term1, float term2, float term3,
                     float term4) {
    shared_ptr<ImageDocument> queryDoc = CreateSimpleDoc(term0, term1,
                                                         term2, term3, term4);
    index->Search(*queryDoc, &results);
  }
  
  // Expect a document to be equal to a set of values
  void ExpectDoc(const ImageDocument* doc, float term0, float term1,
                 float term2, float term3, float term4) {
    SCOPED_TRACE(testing::Message() << "Expecting document to be ("
                 << term0 << "," << term1 << ","
                 << term2 << "," << term3 << ","
                 << term4 << ") but was ("
                 << doc->GetVal(0) << ','
                 << doc->GetVal(1) << ','
                 << doc->GetVal(2) << ','
                 << doc->GetVal(3) << ','
                 << doc->GetVal(4) << ')');

    ExpectTermEq(doc->GetVal(0), term0);
    ExpectTermEq(doc->GetVal(1), term1);
    ExpectTermEq(doc->GetVal(2), term2);
    ExpectTermEq(doc->GetVal(3), term3);
    ExpectTermEq(doc->GetVal(4), term4);
  
  }

  void ExpectTermEq(float docVal, float termVal) {
    if (termVal > 0) {
      EXPECT_FLOAT_EQ(termVal, docVal);
    }
  }

};

TEST_F(ImageSearchIndexTest, EmptyIndex) {
  SearchWithDoc(2, -1, -1, -1, -1);
  ASSERT_EQ(0u, results.size());

  index->BuildIndex();
  SearchWithDoc(2, -1, -1, -1, -1);
  ASSERT_EQ(0u, results.size());
}

TEST_F(ImageSearchIndexTest, FindIdenticalDoc) {
  AddSimpleDoc(2, -1, 4, -1, -1, "A");
  AddSimpleDoc(-1, 3, -1, -1, -1, "B");
  AddSimpleDoc(-1, -1, 1, -1, -1, "C");

  // The index hasn't been built, so this shouldn't return a result
  SearchWithDoc(2, -1, 4, -1, -1);
  ASSERT_EQ(0u, results.size());
  
  index->BuildIndex();
  SearchWithDoc(2, -1, 4, -1, -1);
  ASSERT_GE(results.size(), 1u);
  EXPECT_FLOAT_EQ(results[0].score(), 1.0);
  EXPECT_EQ(results[0].doc()->label(), "A");
}

// Make sure that the tdf-idf cosine calculation is done correctly
TEST_F(ImageSearchIndexTest, TdfIdfCosineTest) {
  AddSimpleDoc(2, -1, 4, -1, -1, "A");
  AddSimpleDoc(-1, 3, -1, -1, -1, "B");
  AddSimpleDoc(-1, -1, 10, -1, -1, "C");

  index->BuildIndex();
  SearchWithDoc(1, 4, -1, -1, -1);

  // Expected values calculated by hand
  EXPECT_EQ(2u, results.size());
  EXPECT_NEAR(results[0].score(), 0.9701, 1e-4);
  EXPECT_EQ(results[0].doc()->label(), "B");
  EXPECT_NEAR(results[1].score(), 0.1951, 1e-4);
  EXPECT_EQ(results[1].doc()->label(), "A");
}

class ArchiveTest : public ImageSearchIndexTest {
public:
  ArchiveTest() : buffer() {}

protected:
  scoped_ptr<stringstream> buffer;

  virtual void SetUp() {
    ImageSearchIndexTest::SetUp();
    buffer.reset(new stringstream(ios_base::out | ios_base::in));
  }

  void WriteToBuf(const ImageSearchIndex& index) {
    xml_oarchive archive(*buffer);
    archive << BOOST_SERIALIZATION_NVP(index);
  }

  shared_ptr<ImageSearchIndex> ReadFromBuf() {
    xml_iarchive archive(*buffer);
    shared_ptr<ImageSearchIndex> retval(new ImageSearchIndex());
    archive >> serialization::make_nvp("index", *retval);

    return retval;
  }

  // Checks that two documents are equal
  void ExpectEqDoc(const ImageDocument& a, const ImageDocument& b) {
    EXPECT_EQ(a.IsNormalized(), b.IsNormalized());
    EXPECT_EQ(a.label(), b.label());
    ASSERT_EQ(a.nUniqueTerms(), b.nUniqueTerms());

    for (ImageDocument::TermMap::const_iterator i = a.begin();
         i != a.end(); ++i) {
      EXPECT_FLOAT_EQ(i->second->val(), b.GetVal(i->first));
      const deque<Term::Coord>* aCoords = i->second->coords();
      const deque<Term::Coord>* bCoords = b.GetCoords(i->first);
      ASSERT_EQ(aCoords == NULL, bCoords == NULL);
      if (bCoords != NULL) {
        EXPECT_TRUE(*aCoords == *bCoords);
      }
    }
  }

  template <typename T>
  shared_ptr<ImageDocument> CreateRandomDoc(T& generator) {
    shared_ptr<ImageDocument> doc (new ImageDocument());
    for (int j=0; j < 5; j++) {
      if (generator() < 0.3) {
        doc->AddTerm(j, generator());
      }
    }
    return doc;
  }

};

// Test that builds an index then does a round trip serialization
// and makes sure that the search works the same.
TEST_F(ArchiveTest, StressArchive) {
  static const int INDEX_SIZE = 1000;
  static const int N_QUERIES = 100;
  
  // setup the random number generator
  mt19937 randEngine(13654);
  uniform_real<> dist(0, 1);
  variate_generator<mt19937&, uniform_real<> > generator(
    randEngine, dist);

  // Now build the index
  for (int i = 0; i < INDEX_SIZE; ++i) {
    index->AddDocument(CreateRandomDoc(generator));
  }
  index->BuildIndex();

  // Now do the round trip
  WriteToBuf(*index);
  shared_ptr<ImageSearchIndex> copy = ReadFromBuf();

  // Finally, issue random queries and check the results
  for (int i = 0; i < N_QUERIES; ++i) {
    vector<SearchResult> origResults;
    vector<SearchResult> copyResults;

    shared_ptr<ImageDocument> queryDoc = CreateRandomDoc(generator);
    index->Search(*queryDoc, &origResults);
    sort_heap(origResults.begin(), origResults.end());
    copy->Search(*queryDoc, &copyResults);
    sort_heap(copyResults.begin(), copyResults.end());


    ASSERT_EQ(origResults.size(), copyResults.size());
    for (unsigned int i = 0; i < origResults.size(); ++i) {
      EXPECT_FLOAT_EQ(origResults[i].score(), copyResults[i].score());
      ExpectEqDoc(*origResults[i].doc(), *copyResults[i].doc());
    }
  }
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
