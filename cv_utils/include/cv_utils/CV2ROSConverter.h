// Helper functions to convert from OpenCV data structures to ROS and
// vice versa
//
// Author: Mark Desnoyer (mdesnoyer@gmail.com)
// Copyright 2012 Carnegie Mellon University
#ifndef __CV2ROS_CONVERTER_H__
#define __CV2ROS_CONVERTER_H__

#include <sensor_msgs/RegionOfInterest.h>
#include <opencv2/core/core.hpp>
#include <vector>

namespace cv_utils {
// Converts RegionOfInterest messages to Rectangles
void ROIs2Rects(const std::vector<sensor_msgs::RegionOfInterest>& rois,
                std::vector<cv::Rect>* rects);
void Rects2ROIs(const std::vector<cv::Rect>& rects,
                std::vector<sensor_msgs::RegionOfInterest>* rois);

} // namespace

#endif // __CV2ROS_CONVERTER_H__
