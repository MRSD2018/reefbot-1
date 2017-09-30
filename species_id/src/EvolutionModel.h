// Model for how a probability distribution changes over time
//
// Author: Mark Desnoyer (mdesnoyer@gmail.com)
// Date: July 2011

#ifndef __EVOLUTION_MODEL_H__
#define __EVOLUTION_MODEL_H__

#include <opencv2/core/core.hpp>
#include "visual_utility/TransformEstimator.h"

namespace species_id {

// Abstract EvolutionModel
class EvolutionModel {
public:
  EvolutionModel() {}
  virtual ~EvolutionModel();

  // Generate the current estimate of the probability distribution
  //
  // Inputs:
  // transformEstimator - object used to do the transforms on the image
  // transform - The transform from the last time AddDistribution was called to now
  // time - The time now in seconds
  // shape - The (rows, cols) shape of the desired distribution model
  virtual cv::Mat_<double> GenerateEstimate(
    const visual_utility::TransformEstimator& transformEstimator,
    cv::Mat transform,
    double time,
    cv::Size2i shape) const = 0;

  // Add a measured probability distrubtion to the model.
  //
  // Inputs:
  // distribution - image of the current probability distribution. Can be modified by the caller
  // time - the time the distribution is valid in seconds
  virtual void AddDistribution(const cv::Mat_<double>& distribution,
                               double time)=0;

  // Reset the model
  virtual void Reset()=0;

};

class GaussianEvolutionModel : public EvolutionModel {
 public:
  GaussianEvolutionModel(double xsigma,
                         double ysigma,
                         double alpha)
    : EvolutionModel(), xsigma_(xsigma), ysigma_(ysigma), alpha_(alpha),
      lastDistribution_(0,0) {}

  virtual ~GaussianEvolutionModel();

  virtual cv::Mat_<double> GenerateEstimate(
    const visual_utility::TransformEstimator& transformEstimator,
    cv::Mat transform,
    double time,
    cv::Size2i shape) const;

  virtual void AddDistribution(const cv::Mat_<double>& distribution,
                               double time);

  virtual void Reset();

 private:
  double xsigma_;
  double ysigma_;
  double alpha_;

  cv::Mat_<double> lastDistribution_;
  double lastTime_;

};

} // namespace

#endif
