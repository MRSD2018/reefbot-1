// A keypoint detector that chooses random points in the image
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: July 2010

#include "RandomDetector.h"

using namespace std;
using namespace boost;
using namespace cv;

namespace species_id {

RandomDetector::~RandomDetector() {}

void RandomDetector::read(const cv::FileNode& fn) {
  fracPixels_ = fn["fracPixels"];
  int seed = fn["seed"];
  seed_ = seed;
  randEngine_ = boost::mt19937(seed_);
}

void RandomDetector::write(cv::FileStorage& fs) const {
  fs << "fracPixels" << fracPixels_;
  fs << "seed" << (int)seed_;
}

void RandomDetector::detectImpl(const Mat& image,
                                vector<KeyPoint>& keypoints,
                                const Mat& mask) const {
  detectImpl(image, mask, keypoints);
}

void RandomDetector::detectImpl(const Mat& image, const Mat& mask,
                                vector<KeyPoint>& keypoints) const {
  // To randomly pick the pixels, we are going to walk through each
  // element and make the probability of selection =
  // numNeeded/numLeft and stop once we've got enough.
  long nPixels = ((long)image.rows) * image.cols;
  double nNeeded = nPixels * fracPixels_;
  double nLeft = nPixels;
  
  uniform_real<> dist(0, 1);
  variate_generator<mt19937&, uniform_real<> > generator(
    randEngine_, dist);

  for (int y = 0; y < image.rows && nNeeded >= 1; y++) {
    for (int x = 0; x < image.cols && nNeeded >= 1; x++) {
      // Choose if this pixel should be selected
      if (generator() <= nNeeded/nLeft) {
        keypoints.push_back(KeyPoint(x, y, 1));
        nNeeded--;
      }
      nLeft--;
    }
  }
}

}
