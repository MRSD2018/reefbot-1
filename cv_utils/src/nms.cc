// Non-maximal suppression routines for object detection.
//
// Author: Mark Desnoyer (mdesnoyer@gmail.com)
// Date: March 2012
#include "cv_utils/nms.h"

#include "ros/ros.h"

#include <algorithm>
#include <math.h>

using namespace std;

namespace cv_utils {

// Prototypes

// Calculates the overlap fraction between two boxes. It is defined as:
// overlap = (area of intersection) / (area of union)
double CalculateOverlap(const cv::Rect& a, const cv::Rect& b);

// End Prototypes

typedef pair<double, cv::Rect> BoxScorePair;

struct BoxScorePairLessThan {
  bool operator()(const BoxScorePair& a, const BoxScorePair& b) {
    return a.first < b.first;
  }
};

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
// overlapThresh - Area overlap threshold for two boxes being similar.
//                 Must be between 0 and 1.0
void ApplyGreedyNonMaximalSuppression(vector<cv::Rect>* boxes,
                                      vector<double>* scores,
                                      double overlapThresh) {
  ROS_ASSERT(boxes);
  ROS_ASSERT(scores);
  ROS_ASSERT(boxes->size() == scores->size());

  // Start by putting the rectangles and scores together in a data
  // structure that we will sort.
  vector<BoxScorePair> sortingBox;
  for (unsigned int i = 0u; i < boxes->size(); ++i) {
    sortingBox.push_back(BoxScorePair((*scores)[i], (*boxes)[i]));
  }

  // Sorts into ascending order
  sort(sortingBox.begin(), sortingBox.end(), BoxScorePairLessThan());

  // Now reset the outputs since we'll fill them as we go
  boxes->clear();
  scores->clear();
  
  // Starting from the highest score, add all those entries that don't
  // overlap with an entry already being returned.
  for (vector<BoxScorePair>::const_reverse_iterator i = sortingBox.rbegin();
       i != sortingBox.rend(); ++i) {
    bool doesOverlap = false;
    for (vector<cv::Rect>::const_iterator j = boxes->begin();
         j != boxes->end(); ++j) {
      if (CalculateOverlap(i->second, *j) > overlapThresh) {
        doesOverlap = true;
        break;
      }
    }

    if (!doesOverlap) {
      boxes->push_back(i->second);
      scores->push_back(i->first);
    }
  }
}

double CalculateOverlap(const cv::Rect& a, const cv::Rect& b) {
  // Calculate the intersection box
  int x1 = MAX(a.x, b.x);
  int y1 = MAX(a.y, b.y);
  int x2 = MIN(a.x + a.width, b.x + b.width);
  int y2 = MIN(a.y + a.height, b.y + b.height);
  int w = x2 - x1;
  int h = y2 - y1;

  if (h <= 0 || w <= 0) {
    return 0.0;
  }

  double intersection = w * h;

  return intersection / (a.width*a.height + b.width*b.height - intersection);
}

} // namespace
