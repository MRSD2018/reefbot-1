// Module that extracts objects that move significantly relative to
// the background in an image sequence.
//
// ROS Input Type: sensor_msgs::Image
// ROS Output Type: reefbot_msgs::SpeciesIDResponse
//
// Author: Mark Desnoyer markd@cmu.edu
// Date: June 2011
#ifndef __MOTION_EXTRACTOR_H__
#define __MOTION_EXTRACTOR_H__

#include <opencv2/core/core.hpp>
#include <boost/scoped_ptr.hpp>
#include "cv_blobs/BlobResult.h"
#include "visual_utility/TransformEstimator.h"
#include "EvolutionModel.h"
#include "visual_utility/VisualUtilityEstimator.h"

namespace species_id {

class MotionExtractor {
public:
  // Constructor
  //
  // Inputs:
  // transformEstimator - Model for how to transform images from one frame to the next
  // evolutionModel - Model object for evolution the probability prior. Takes ownership
  // confidenceThreshold - The probability threshold for identifying an object
  // distThreshold - Distance in LAB color space to be considered an object. This is the threshold in a sigmoid function.
  // distDecay - The sharpness of the sigmoid function
  // minObjSize - The minimum size in pixels for an object
  // alpha - Mixing parameter for the prior vs. the current distance. Larger values give prefernce to the current distance.
  MotionExtractor(const visual_utility::TransformEstimator& transformEstimator,
                  EvolutionModel* evolutionModel,
                  double confidenceThreshold,
                  double paretoThreshold,
                  double distDecay,
                  int64 minObjSize,
                  double alpha)
    : time_(-1e4),
      vuEstimator_(transformEstimator, paretoThreshold, distDecay, 1),
      evolutionModel_(evolutionModel),
      transformEstimator_(transformEstimator),
      confidenceThreshold_(confidenceThreshold),
      minObjSize_(minObjSize),
      alpha_(alpha){}

  ~MotionExtractor();

  // Adds a new image to the motion model that was taken at a specific time
  void AddImage(const cv::Mat& image, double time);

  // Retrieves the currently known objects
  const cv_blobs::BlobResult<float>& RetrieveObjects() const {
    return objects_;
  };

  double time() const { return time_; }

  const cv::Point& maxLocation() const { return maxLocation_; }

private:
  // Time at which the state in objects_ was known
  double time_;

  visual_utility::LABMotionVUEstimator vuEstimator_;
  boost::scoped_ptr<EvolutionModel> evolutionModel_;
  const visual_utility::TransformEstimator& transformEstimator_;

  // Parameters for the algorithm

  // Overall confidence threshold for identifying an object
  double confidenceThreshold_;
  // The minimum size of an object in pixels
  int64 minObjSize_;
  //Mixing parameter for combining the prior estimate of the motion
  //with the current measurement. Smaller values shift the bias to the prior
  double alpha_;
  

  // The last image input into the model both in LAB space and in greyscale
  cv::Mat_<cv::Vec3d> lastLabImage_;
  cv::Mat_<double> lastGreyImage_;

  // List of known objects in the scene
  cv_blobs::BlobResult<float> objects_;

  // Location of the highest likelihood
  cv::Point maxLocation_;
};

}

#endif // __MOTION_EXTRACTOR_H__
