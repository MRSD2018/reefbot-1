// A keypoint detector that chooses random points in the image
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: July 2010

#ifndef _SPECIES_ID_RANDOM_DETECTOR_H__
#define _SPECIES_ID_RANDOM_DETECTOR_H__

#include <boost/random.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <vector>

namespace species_id {

// This detector picks points randomly in the image. It is guaranteed
// not to pick two identical points.
class RandomDetector : public cv::FeatureDetector {

public:
  // Constructor.
  //
  // Inputs:
  // fracPixels - Fraction of pixels to return
  // seed - Seed for the random number generator
  RandomDetector(double fracPixels=1.0, uint32_t seed=12346)
    : cv::FeatureDetector(), fracPixels_(fracPixels), seed_(seed), randEngine_(seed) {}

  virtual ~RandomDetector();

  virtual void read(const cv::FileNode& fn);
  virtual void write(cv::FileStorage& fs) const;

private:
  double fracPixels_;
  uint32_t seed_;
  // Mutable because we want different random numbers for new images
  mutable boost::mt19937 randEngine_;

protected:
  virtual void detectImpl(const cv::Mat& image, const cv::Mat& mask,
                          std::vector<cv::KeyPoint>& keypoints) const;

  virtual void detectImpl(const cv::Mat& image,
                          std::vector<cv::KeyPoint>& keypoints,
                          const cv::Mat& mask=cv::Mat() ) const;

};

}

#endif // _SPECIES_ID_RANDOM_DETECTOR_H__
