#include "HistogramDescriptorGenerator-Inl.h"
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "opencv2/core/core.hpp"
#include "ImageDescriptorGenerator-Inl.h"
#include "ImageDescriptor-Inl.h"
#include "reefbot_msgs/ImageRegion.h"

using namespace std;
using namespace boost;
using namespace cv;

namespace species_id {

// A mock class that just returns whatever entries were loaded into
// it. It doesn't do multiple masks.
class DescriptorGeneratorMock : public ImageDescriptorGenerator<float> {
public:

  virtual ~DescriptorGeneratorMock() {}

  void AddEntry(float a, float b, float c, float x, float y) {
    vector<float> entry;
    entry.push_back(a);
    entry.push_back(b);
    entry.push_back(c);
    collection.push_back(shared_ptr<ImageDescriptor<float> >(
      new ImageDescriptor<float>(
        cv::Point2f(x,y),
        &entry)));
  }

  void ExtractUsingMasks(
    const cv::Mat& image,
    const MaskCollection& masks,
    vector<shared_ptr<DescriptorCollection> >* descriptors) const {
    descriptors->push_back(shared_ptr<DescriptorCollection>(
      new DescriptorCollection(collection)));      
  }
  
private:
  DescriptorCollection collection;

  virtual void ExtractFromWholeImage(
    const cv::Mat& image,
    const std::vector<cv::KeyPoint>& keypoints,
    DescriptorCollection* descriptors) const {}
};

class HistogramTest : public ::testing::Test {
  protected:
  string dataBuf_;
  stringstream data_;
  DescriptorGeneratorMock rawGen_;
  scoped_ptr<HistogramDescriptorGenerator<float> > generator_;
  vector<reefbot_msgs::ImageRegion> masks_;

  HistogramTest()
    : dataBuf_(), data_(dataBuf_), rawGen_(), generator_(NULL) {}

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    static char sp = ' ';

    // Load up the data for the dictionary which is 3D vectors
    data_ << 3 << endl
          << 5 << endl
          << 5 << sp << 2 << sp << 0 << endl
          << 0 << sp << 5 << sp << 0 << endl
          << 0 << sp << 0 << sp << 5 << endl
          << 1 << sp << 1 << sp << 1 << endl
          << 3 << sp << 3 << sp << 0 << endl;

    // Build up the dictionary based generator
    generator_.reset(
      HistogramDescriptorGenerator<float>::CreateFromINRIADictionary(
        &rawGen_,
        data_));

    masks_.push_back(reefbot_msgs::ImageRegion());
  }
};



TEST_F(HistogramTest, EmptyInitially) {
  vector<shared_ptr<HistogramDescriptorGenerator<float>::DescriptorCollection> > collection;
  
  generator_->ExtractUsingMasks(Mat(), masks_, &collection);

  // Should return an all zeros histogram
  ASSERT_EQ(collection.size(), 1u);
  ASSERT_EQ(collection[0]->size(), 1u);
  ASSERT_EQ((*collection[0])[0]->DescriptorSize(), 5);
  for (int i = 0; i < 5; i++) {
    EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(i), 0);
  }
}

TEST_F(HistogramTest, AllOneEntry) {
  vector<shared_ptr<HistogramDescriptorGenerator<float>::DescriptorCollection> > collection;

  rawGen_.AddEntry(0, 4, 0, 0, 0);
  rawGen_.AddEntry(0, 6, 0, 0, 1);
  
  generator_->ExtractUsingMasks(Mat(), masks_, &collection);

  // Should return an a histogram with all the entries in one bucket
  ASSERT_EQ(collection.size(), 1u);
  ASSERT_EQ(collection[0]->size(), 1u);
  ASSERT_EQ((*collection[0])[0]->DescriptorSize(), 5);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(0), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(1), 1.0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(2), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(3), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(4), 0);
}

TEST_F(HistogramTest, ANiceMixOfEntries) {
  vector<shared_ptr<HistogramDescriptorGenerator<float>::DescriptorCollection> > collection;

  rawGen_.AddEntry(0, 4, 0, 0, 0);
  rawGen_.AddEntry(0, 0, 7, 0, 1);
  rawGen_.AddEntry(1, 2, 1, 1, 0);
  rawGen_.AddEntry(0, 6, 0, 1, 1);
  
  generator_->ExtractUsingMasks(Mat(), masks_, &collection);

  // Should return an a histogram spread between 3 buckets
  ASSERT_EQ(collection.size(), 1u);
  ASSERT_EQ(collection[0]->size(), 1u);
  ASSERT_EQ((*collection[0])[0]->DescriptorSize(), 5);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(0), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(1), 0.5);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(2), 0.25);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(3), 0.25);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(4), 0);
}

TEST_F(HistogramTest, NormSquaredDistanceUsed) {
  vector<shared_ptr<HistogramDescriptorGenerator<float>::DescriptorCollection> > collection;

  rawGen_.AddEntry(5, 5, 0, 0, 0);
  
  generator_->ExtractUsingMasks(Mat(), masks_, &collection);

  ASSERT_EQ(collection.size(), 1u);
  ASSERT_EQ(collection[0]->size(), 1u);
  ASSERT_EQ((*collection[0])[0]->DescriptorSize(), 5);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(0), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(1), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(2), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(3), 0);
  EXPECT_FLOAT_EQ((*collection[0])[0]->GetVal(4), 1.0);
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
