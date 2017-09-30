#include "cv_utils/nms.h"

#include <gtest/gtest.h>

using cv::Rect;
using std::vector;

namespace cv_utils {

class GreedyNMSTest : public ::testing::Test {
 protected:
  vector<double> scores;
  vector<Rect> boxes;

  virtual void SetUp() {
    boxes.clear();
    scores.clear();
  }

  void AddBox(int x, int y, int w, int h, double score) {
    scores.push_back(score);
    boxes.push_back(Rect(x, y, w, h));
  }

  void DoNMS(double thresh) {
    ApplyGreedyNonMaximalSuppression(&boxes, &scores, thresh);
  }

  // Returns true if the box is in the set
  bool BoxInSet(int x, int y, int w, int h) {
    Rect toFind(x, y, w, h);

    for(vector<Rect>::const_iterator i = boxes.begin();
        i != boxes.end(); ++i) {
      if (*i == toFind) {
        return true;
      }
    }
    return false;
  }
  
};

TEST_F(GreedyNMSTest, NoBoxes) {
  EXPECT_FALSE(BoxInSet(0, 0, 2, 3));
}

TEST_F(GreedyNMSTest, SingleBox) {
  AddBox(0, 0, 2, 3, 5.0);

  DoNMS(0.0);

  EXPECT_TRUE(BoxInSet(0, 0, 2, 3));
}

TEST_F(GreedyNMSTest, NoOverlap) {
  AddBox(0, 0, 2, 3, 5.0);
  AddBox(20, 30, 2, 3, 4.0);

  DoNMS(0.0);

  EXPECT_TRUE(BoxInSet(0, 0, 2, 3));
  EXPECT_TRUE(BoxInSet(20,30, 2, 3));
}

TEST_F(GreedyNMSTest, OverlapTooSmall) {
  AddBox(0, 0, 10, 10, 5.0);
  AddBox(1, 2, 7, 7, 4.0);

  DoNMS(0.5);

  EXPECT_TRUE(BoxInSet(0, 0, 10, 10));
  EXPECT_TRUE(BoxInSet(1, 2, 7, 7));
}

TEST_F(GreedyNMSTest, EnoughOverlap) {
  AddBox(0, 0, 10, 10, 5.0);
  AddBox(1, 2, 7, 7, 4.0);

  DoNMS(0.45);

  EXPECT_TRUE(BoxInSet(0, 0, 10, 10));
  EXPECT_FALSE(BoxInSet(1, 2, 7, 7));
}

TEST_F(GreedyNMSTest, EnoughOverlapNotSubset) {
  AddBox(0, 0, 10, 10, 5.0);
  AddBox(4, 5, 6, 7, 4.0);

  DoNMS(29.0/112.0);

  EXPECT_TRUE(BoxInSet(0, 0, 10, 10));
  EXPECT_FALSE(BoxInSet(4, 5, 6, 7));
}

TEST_F(GreedyNMSTest, TwoOverlapOneNot) {
  AddBox(0, 0, 10, 10, 5.0);
  AddBox(1, 2, 7, 7, 6.0);
  AddBox(15, 18, 3, 4, 10);

  DoNMS(0.45);

  EXPECT_FALSE(BoxInSet(0, 0, 10, 10));
  EXPECT_TRUE(BoxInSet(1, 2, 7, 7));
  EXPECT_TRUE(BoxInSet(15, 18, 3, 4));
}


} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
