#include "DocSimilarityCalculator.h"
#include <gtest/gtest.h>

#include <boost/scoped_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace boost;

namespace species_id {

class PerspectiveGeometryTest : public ::testing::Test {
public:
  PerspectiveGeometryTest()
    : reranker(5.0) {}

protected:
  PerspectiveGeometricReweight reranker;
  scoped_ptr<ImageDocument> docA;
  scoped_ptr<ImageDocument> docB;

  void SetUp() {
    // Create two documents and load four points into the first document
    docA.reset(new ImageDocument());
    docA->AddTerm(1, 1.0, 30.0, 30.0);
    docA->AddTerm(2, 1.0, 25.0, 23.1);
    docA->AddTerm(3, 1.0, 34.7, 28.0);
    docA->AddTerm(4, 1.0, 10.6, 40.1);
    docA->AddTerm(5, 1.0, 60.1, 23.8);

    docB.reset(new ImageDocument());
  }

  void AddTerm(ImageDocument* doc, TermID termId, Point2f point) {
    doc->AddTerm(termId, 1.0, point.x, point.y);
  }

  // Do a perspective transform of point using the 3x3 matrix M
  Point2f TransformPoint(Point2f point, const Mat_<float>& M) {
    float t = M[2][0]*point.x + M[2][1]*point.y + M[2][2];
    return Point2f((M[0][0]*point.x + M[0][1]*point.y + M[0][2])/t,
                   (M[1][0]*point.x + M[1][1]*point.y + M[1][2])/t);
                   
  }
  Point2f TransformPoint(Term::Coord point, const Mat_<float>& M) {
    return TransformPoint(Point2f(point.first, point.second), M);
  }

  void AddTransformedPoints(ImageDocument* docToMod, const Mat_<float>& M,
                            const ImageDocument& docWithPoints) {
    for (ImageDocument::TermMap::const_iterator i = docWithPoints.begin();
         i != docWithPoints.end(); ++i) {
      AddTerm(docToMod, i->first, TransformPoint((*i->second->coords())[0], M));
    }
  }
};

// If there are no common terms, the doc similarity should not change
TEST_F(PerspectiveGeometryTest, NoCommonTerms) {
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
}

// If there are less than 6 common terms, we can't do a perspective
// transform, so the similarity shouldn't change.
TEST_F(PerspectiveGeometryTest, LessThan5CommonTerms) {
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
  docA->AddTerm(1, 1.0, 30.0, 30.0);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
  docB->AddTerm(2, 1.0, 25.0, 23.1);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
  docB->AddTerm(3, 1.0, 34.7, 28.0);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
  docB->AddTerm(4, 1.0, 10.6, 40.1);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
  docB->AddTerm(5, 1.0, 60.1, 23.8);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
  
}

// No transformation and 6 common terms should have one inlier
TEST_F(PerspectiveGeometryTest, SixCommonTermsIdentityTransform) {
  docB->AddTerm(1, 1.0, 30.0, 30.0);
  docB->AddTerm(2, 1.0, 25.0, 23.1);
  docB->AddTerm(3, 1.0, 34.7, 28.0);
  docB->AddTerm(4, 1.0, 10.6, 40.1);
  docB->AddTerm(5, 1.0, 60.1, 23.8);

  docA->AddTerm(6, 1.0, 20.1, 13.8);
  docB->AddTerm(6, 1.0, 20.1, 13.8);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 1.32);
}

// No transformation and 5 common terms should have one inlier
TEST_F(PerspectiveGeometryTest, OneOutlierIdentityTransform) {
  docB->AddTerm(1, 1.0, 30.0, 30.0);
  docB->AddTerm(2, 1.0, 25.0, 23.1);
  docB->AddTerm(3, 1.0, 34.7, 28.0);
  docB->AddTerm(4, 1.0, 10.6, 40.1);
  docB->AddTerm(5, 1.0, 60.1, 23.8);

  docA->AddTerm(6, 1.0, 20.1, 13.8);
  docB->AddTerm(6, 1.0, 66.1, 3.8);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
}

TEST_F(PerspectiveGeometryTest, TwoCoordsOnTerm) {
  docB->AddTerm(1, 1.0, 30.0, 30.0);
  docB->AddTerm(2, 1.0, 25.0, 23.1);
  docB->AddTerm(3, 1.0, 34.7, 28.0);
  docB->AddTerm(4, 1.0, 10.6, 40.1);
  docB->AddTerm(5, 1.0, 60.1, 23.8);

  docA->AddTerm(6, 1.0, 20.1, 13.8);
  docB->AddTerm(6, 1.0, 66.1, 3.8); // An outlier
  docB->AddTerm(6, 1.0, 20.1, 13.8); // An inlier
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 1.32);
}

TEST_F(PerspectiveGeometryTest, PointsOnALine) {
  docA.reset(new ImageDocument());
  docA->AddTerm(1, 1.0, 30.0, 30.0);
  docA->AddTerm(2, 1.0, 30.0, 23.1);
  docA->AddTerm(3, 1.0, 30.0, 28.0);
  docA->AddTerm(4, 1.0, 30.0, 40.1);
  docA->AddTerm(5, 1.0, 30.0, 1.0);
  docA->AddTerm(6, 1.0, 30.0, 11.0);

  Mat_<float> M = Mat_<float>::zeros(3,3);
  M[0][0] = 0.5;
  M[1][1] = 2.0;
  M[2][2] = 0.8;

  AddTransformedPoints(docB.get(), M, *docA);
  EXPECT_DOUBLE_EQ(reranker.CalcNewSimilarity(*docA, *docB, 0.32), 0.32);
}


} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
