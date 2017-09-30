#include "ros/ros.h"
#include "MotionExtractor.h"
#include <string>

using namespace ros;
using namespace std;

MotionExtractor motionExtractor;

void HandleNewImage(const sensor_msgs::Image::ConstPtr& msg) {
  IplImage* iplImage;
  try {
    iplImage = sensor_msgs::CvBridge().imgMsgToCv(msg, "bgr8");
  } catch (sensor_msgs::CvBridgeException error) {
    ROS_ERROR("Could not import the image");
    return;
  }

  cv::Mat cvImage(iplImage);

  motionExtractor.AddImage(cvImage, msg.header.time.ToSecs());
}

void HandleNewImageRequest(const reefbot_msgs::ImageCaptured::ConstPtr& msg) {
  if (motionExtractor.time() - msg.header.time.ToSecs() < 0.1) {
    // We have a good track of the objects, so send the request to the identifier
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "MotionFishExtractor");

  NodeHandle handle();
  NodeHandle local("~");

  double diffThreshold;
  double mergeThreshold;
  int maxIter;
  float minPrecision;
  string imageTopic;
  string imageCapturedTopic;
  

  local.param("diff_threshold", diffThreshold, 1000);
  // TODO(mdesnoyer): Change this to be more reasonable once I can actually merge stuff
  local.param("merge_threshold", mergeThreshold, 1e3);
  local.param("max_iter", maxIter, 100);
  local.param("min_precision", minPrecision, 1e-6);

  local.param<string>("image_topic", imageTopic, "MotionFishExtractor/images");
  Subscriber imageListener = handle.subscribe(imageTopic, 8, HandleNewImage);

  local.param<string>("image_captured_topic", imageCapturedTopic, "image_captured");
  Subscriber captureListener = handle.subscribe(imageCapturedTopic, 8, HandleNewImageRequest);

  local.param<string>("label_request_topic", labelRequestTopic, "label_request_topic");

  spin();

}
