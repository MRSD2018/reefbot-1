#include "cv_bridge/cv_bridge.h"
#include <boost/scoped_ptr.hpp>
#include <gtest/gtest.h>
#include "opencv2/core/core.hpp"
#include <sstream>

using namespace cv;
using namespace boost;
using namespace std;

namespace cv_bridge {

class MatNDTest : public ::testing::Test {
 protected:
  Mat mat_;

  virtual void SetUp() {
    int size[] = {2,3,4};
    mat_ = Mat(3, size, CV_8U, Scalar::all(0));

    // Fill the matrix with the linear index number
    for (uint8_t i = 0; i < 24; ++i) {
      mat_.at<uint8_t>(0, 0, i) = i;
    }
  }
};

TEST_F(MatNDTest, RoundTripTestCopyNoConversion) {
  CvMultiMat startMat;
  startMat.mat = mat_;
  
  sensor_msgs::MatNDPtr msg = startMat.toMsg();

  CvMultiMatPtr matCopy = toCvCopy(msg);

  // Make sure the layout of the matrices is the same
  ASSERT_EQ(mat_.dims, matCopy->mat.dims);
  for (int i = 0; i < mat_.dims; ++i) {
    ASSERT_EQ(mat_.size[i], matCopy->mat.size[i]);
  }

  // Make sure all the values are the same  
  MatConstIterator_<uint8_t> copyI = matCopy->mat.begin<uint8_t>();
  for (MatConstIterator_<uint8_t> startI = mat_.begin<uint8_t>();
       startI != mat_.end<uint8_t>();
       ++startI, ++copyI) {
    EXPECT_EQ(*startI, *copyI);
  }

  // Make sure that the locations are where I expect them
  EXPECT_EQ(mat_.at<uint8_t>(0, 1, 2), 6);
  EXPECT_EQ(mat_.at<uint8_t>(1, 0, 1), 13);
  EXPECT_EQ(mat_.at<uint8_t>(1, 2, 0), 20);
  
  // Make sure that a copy was made
  ASSERT_NE(&msg->data[0],
            matCopy->mat.data);
}

TEST_F(MatNDTest, RoundTripTest32BitInt) {
  CvMultiMat startMat;
  mat_.convertTo(startMat.mat, CV_32S);
  
  sensor_msgs::MatNDPtr msg = startMat.toMsg();

  CvMultiMatPtr matCopy = toCvCopy(msg);
  ASSERT_EQ(matCopy->mat.type(), CV_32S);

  // Make sure the layout of the matrices is the same
  ASSERT_EQ(mat_.dims, matCopy->mat.dims);
  for (int i = 0; i < mat_.dims; ++i) {
    ASSERT_EQ(mat_.size[i], matCopy->mat.size[i]);
  }


  // Make sure all the values are the same  
  MatConstIterator_<int32_t> copyI = matCopy->mat.begin<int32_t>();
  for (MatConstIterator_<int32_t> startI = startMat.mat.begin<int32_t>();
       startI != startMat.mat.end<int32_t>();
       ++startI, ++copyI) {
    EXPECT_EQ(*startI, *copyI);
  }

  // Make sure that the locations are where I expect them
  EXPECT_EQ(matCopy->mat.at<int32_t>(0, 1, 2), 6);
  EXPECT_EQ(matCopy->mat.at<int32_t>(1, 0, 1), 13);
  EXPECT_EQ(matCopy->mat.at<int32_t>(1, 2, 0), 20);
  
  // Make sure that a copy was made
  ASSERT_NE(&msg->data[0],
            matCopy->mat.data);
}

TEST_F(MatNDTest, RoundTripTestShareNoConversion) {
  CvMultiMat startMat;
  startMat.mat = mat_;
  sensor_msgs::MatNDPtr msg = startMat.toMsg();

  CvMultiMatConstPtr matCopy = toCvShare(msg);
  
  // Make sure the layout of the matrices is the same
  ASSERT_EQ(mat_.dims, matCopy->mat.dims);
  for (int i = 0; i < mat_.dims; ++i) {
    ASSERT_EQ(mat_.size[i], matCopy->mat.size[i]);
  }

  // Make sure that the data is shared between the message and the copy
  ASSERT_EQ(&msg->data[0],
            matCopy->mat.data);
}

TEST_F(MatNDTest, RoundTripTestCopyConversion) {
  CvMultiMat startMat;
  startMat.mat = mat_;
  
  sensor_msgs::MatNDPtr msg = startMat.toMsg();

  CvMultiMatPtr matCopy = toCvCopy(msg, "32S");

  // Make sure the layout of the matrices is the same
  ASSERT_EQ(mat_.dims, matCopy->mat.dims);
  for (int i = 0; i < mat_.dims; ++i) {
    ASSERT_EQ(mat_.size[i], matCopy->mat.size[i]);
  }

  // Make sure all the values are the same  
  MatConstIterator_<int> copyI = matCopy->mat.begin<int>();
  for (MatConstIterator_<uint8_t> startI = mat_.begin<uint8_t>();
       startI != mat_.end<uint8_t>();
       ++startI, ++copyI) {
    EXPECT_EQ(*startI, *copyI);
  }
}

TEST_F(MatNDTest, CheckMessageFormat) {
  CvMultiMat startMat;
  startMat.mat = mat_;
  
  sensor_msgs::MatNDPtr msg = startMat.toMsg();

  EXPECT_EQ(msg->sizes[0], 2);
  EXPECT_EQ(msg->sizes[1], 3);
  EXPECT_EQ(msg->sizes[2], 4);

  EXPECT_EQ(msg->encoding, string("8U"));

  EXPECT_EQ(msg->is_bigendian, false);

  ASSERT_EQ(msg->data.size(), 24u);
  for (uint32_t i = 0u; i < msg->data.size(); ++i) {
    EXPECT_EQ(msg->data[i], i);
  }
}

} // namespace 

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
