#include "MotionExtractor.h"

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <limits>
#include "cv_blobs/BlobResult-Inline.h"
#include "visual_utility/TransformEstimator-Inl.h"
#include "cv_blobs/BlobFilters.h"
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;
using visual_utility::TransformEstimator;
using namespace cv_blobs;

namespace species_id {

MotionExtractor::~MotionExtractor() {}



void MotionExtractor::AddImage(const Mat& image, double time) {

  Mat_<double> pObjGivDist = vuEstimator_.CalculateVisualUtility(image, time);

  const Mat* M = vuEstimator_.GetLastTransform();
  if (M == NULL) {
    evolutionModel_->Reset();
    return;
  }

  // Next retreive the probability of an object at a given location
  // given the measurement at the last time step. We do this using the
  // evolutionModel
  Mat_<double> pObjGivPrior = evolutionModel_->GenerateEstimate(
    transformEstimator_,
    *M,
    time,
    pObjGivDist.size());

  // Calculate the probability that an object is at a location by
  // combining linearily the two conditional probabilities
  Mat_<double> pObj = alpha_*pObjGivDist + (1-alpha_)*pObjGivPrior;

  // Record the probability estimate in the evolution model
  evolutionModel_->AddDistribution(pObj, time);

  Mat_<float> floatP;
  pObj.assignTo(floatP, CV_32FC1);
  morphologyEx(floatP, floatP, MORPH_CLOSE, Mat_<uchar>::ones(5,5));

  objects_.FindBlobs(floatP, confidenceThreshold_);
  objects_.Filter(AreaCompare<greater<int64>, int64>(minObjSize_));

  // Find the location of maximum likelihood
  minMaxLoc(pObj, NULL, NULL, NULL, &maxLocation_);

  time_ = time;
}



}; // namespace
