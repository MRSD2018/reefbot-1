#include "cv_utils/IntegralHistogram-Inl.h"

#include <gtest/gtest.h>
#include <boost/scoped_ptr.hpp>

using namespace cv;
using namespace boost;

namespace cv_utils {

class IntegralHistTest : public ::testing::Test {
protected:
  Mat_<double> image;

  virtual void SetUp() {
    image = Mat_<double>::zeros(3,4);
    image(2,0) = 4.0;
    image(0,0) = 3.0;
    image(1,3) = 1.0;
    image(0,1) = 3.0;
  }
};

TEST_F(IntegralHistTest, BasicTest) {
  scoped_ptr<IntegralHistogram<double> > hist(
    IntegralHistogram<double>::Calculate(image, 5));

  EXPECT_EQ((*hist)(0,0,0), 0);
  EXPECT_EQ((*hist)(1,1,0), 0);
  EXPECT_EQ((*hist)(3,4,0), 8);
  EXPECT_EQ((*hist)(3,4,1), 1);
  EXPECT_EQ((*hist)(3,4,2), 0);
  EXPECT_EQ((*hist)(3,4,3), 2);
  EXPECT_EQ((*hist)(3,4,4), 1);
  EXPECT_EQ((*hist)(1,2,3), 2);
  EXPECT_EQ((*hist)(1,4,0), 2);
  hist.reset(NULL);
}

TEST_F(IntegralHistTest, GetArrayHistTest) {
  scoped_ptr<IntegralHistogram<double> > hist(
    IntegralHistogram<double>::Calculate(image, 5));
  const cv::Mat_<double> pixHist = hist->GetArrayHist(3,4);
  EXPECT_EQ(pixHist(0), 8);
  EXPECT_EQ(pixHist(1), 1);
  EXPECT_EQ(pixHist(2), 0);
  EXPECT_EQ(pixHist(3), 2);
  EXPECT_EQ(pixHist(4), 1);
}

TEST_F(IntegralHistTest, GetArrayHistPointTest) {
  scoped_ptr<IntegralHistogram<double> > hist(
    IntegralHistogram<double>::Calculate(image, 5));
  const cv::Mat_<double> pixHist = hist->GetArrayHist(Point(4,3));
  EXPECT_EQ(pixHist(0), 8);
  EXPECT_EQ(pixHist(1), 1);
  EXPECT_EQ(pixHist(2), 0);
  EXPECT_EQ(pixHist(3), 2);
  EXPECT_EQ(pixHist(4), 1);
}

TEST_F(IntegralHistTest, GetHistInRegionTest) {
  scoped_ptr<IntegralHistogram<double> > hist(
    IntegralHistogram<double>::Calculate(image, 5));
  const cv::Mat_<double> pixHist = hist->GetHistInRegion(Rect(0, 1, 4, 2));
  EXPECT_EQ(pixHist(0), 6);
  EXPECT_EQ(pixHist(1), 1);
  EXPECT_EQ(pixHist(2), 0);
  EXPECT_EQ(pixHist(3), 0);
  EXPECT_EQ(pixHist(4), 1);
}

TEST_F(IntegralHistTest, WeightedEntryTest) {
  Mat_<double> weights = Mat_<double>::ones(3,4)*3;
  weights(1,3) = 7;
  scoped_ptr<IntegralHistogram<double> > hist(
    IntegralHistogram<double>::Calculate<double>(image,
                                                 5,
                                                 NULL,
                                                 weights));

  const cv::Mat_<double> pixHist = hist->GetArrayHist(3,4);
  EXPECT_EQ(pixHist(0), 24);
  EXPECT_EQ(pixHist(1), 7);
  EXPECT_EQ(pixHist(2), 0);
  EXPECT_EQ(pixHist(3), 6);
  EXPECT_EQ(pixHist(4), 3);
}


TEST_F(IntegralHistTest, LinearInterpTest) {
  image(0,2) = 1.3;

  scoped_ptr<IntegralHistogram<double> > hist(
    IntegralHistogram<double>::Calculate<double>(
      image,
      5,
      NULL,
      Mat_<double>(),
      IntegralHistogram<double>::LINEAR_INTERP));
  

  const cv::Mat_<double> pixHist = hist->GetArrayHist(3,4);
  EXPECT_FLOAT_EQ(pixHist(0), 7);
  EXPECT_FLOAT_EQ(pixHist(1), 1.7);
  EXPECT_FLOAT_EQ(pixHist(2), 0.3);
  EXPECT_FLOAT_EQ(pixHist(3), 2);
  EXPECT_FLOAT_EQ(pixHist(4), 1);
}


} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
