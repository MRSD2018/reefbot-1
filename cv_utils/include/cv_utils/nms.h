// Non-maximal suppression routines for object detection.
//
// Author: Mark Desnoyer (mdesnoyer@gmail.com)
// Date: March 2012

#ifndef CV_UTILS_NMS_H_
#define CV_UTILS_NMS_H_

#include <vector>
#include <opencv2/core/core.hpp>

namespace cv_utils {

// Greedily does non maximal suppression by taking those bounding
// boxes of highest score and removing those above an area overlap
// threshold.
//
// Inputs:
// locs - Bounding boxes specifying locations. Will be changed in place 
//        so that the non-maximal entries are removed
// scores - Scores of all the bounding boxes in locs. Will be changed in 
//          places so that the non-maximal entries are removed. Must be 
//          the same order as locs.
// overlapThresh - Area overlap threshold for two boxes being similar
void ApplyGreedyNonMaximalSuppression(std::vector<cv::Rect>* boxes,
                                      std::vector<double>* scores,
                                      double overlapThresh);

} // namespace

#endif // CV_UTILS_NMS_H_
