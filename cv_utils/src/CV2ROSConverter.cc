#include "cv_utils/CV2ROSConverter.h"
#include <ros/ros.h>

using namespace std;
using namespace cv;
using sensor_msgs::RegionOfInterest;

namespace cv_utils {

void ROIs2Rects(const vector<RegionOfInterest>& rois,
                vector<Rect>* rects) {
  ROS_ASSERT(rects);
  for (vector<RegionOfInterest>::const_iterator i = rois.begin();
       i != rois.end(); ++i) {
    rects->push_back(Rect_<int>(i->x_offset, i->y_offset, i->width,
                                i->height));
  }
}

void Rects2ROIs(const vector<Rect>& rects,
                vector<RegionOfInterest>* rois) {
  ROS_ASSERT(rois);
  for (vector<Rect>::const_iterator i = rects.begin();
       i != rects.end(); ++i) {
    rois->push_back(RegionOfInterest());

    RegionOfInterest& curRoi(rois->back());
    curRoi.x_offset = i->x;
    curRoi.y_offset = i->y;
    curRoi.height = i->height;
    curRoi.width = i->width;
  }
}

} // namespace
