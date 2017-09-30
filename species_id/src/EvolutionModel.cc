#include "EvolutionModel.h"
#include <opencv2/imgproc/imgproc.hpp>

#include "visual_utility/TransformEstimator-Inl.h"

using namespace cv;
using visual_utility::TransformEstimator;

namespace species_id {

EvolutionModel::~EvolutionModel() {}

GaussianEvolutionModel::~GaussianEvolutionModel() {}

Mat_<double> GaussianEvolutionModel::GenerateEstimate(
  const TransformEstimator& transformEstimator,
  Mat transform,
  double time,
  Size2i shape) const {

  if (lastDistribution_.total() == 0) {
    // We haven't had a call to AddDistribution yet, so set every pixel to 0.5
    return Mat_<double>(shape, 0.5);
  }

  double dt = time - lastTime_;
  Mat_<double> estimate;
  GaussianBlur(lastDistribution_, estimate, Size(0,0), xsigma_*dt, ysigma_*dt,
               BORDER_REPLICATE);

  estimate = transformEstimator.ApplyTransform(estimate, transform,
                                               geo::BORDER_CONSTANT,
                                               0.0);
  return alpha_ + (1-alpha_)*estimate;
  
}

void GaussianEvolutionModel::AddDistribution(
  const cv::Mat_<double>& distribution,
  double time) {
  lastTime_ = time;
  lastDistribution_ = distribution.clone();
}

void GaussianEvolutionModel::Reset() {
  lastDistribution_ = Mat_<double>(0,0);
  lastTime_ = -1;
}

} // namespace
