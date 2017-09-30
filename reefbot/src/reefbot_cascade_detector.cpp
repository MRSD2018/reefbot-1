// ROS Node that wraps the OpenCV Haar Cascade Detector for object
// detection for Reefbot usages
//
// Provides a service with reefbot_msgs::FindSpecies
//
// ROS Input Type: reefbot_msgs::ImageCaptured
// ROS Output Type: reefbot_msgs::SpeciesIDResponse
//
// Author: Mark Desnoyer markd@cmu.edu
// Date: May 2011

#include <vector>
#include "ros/ros.h"
#include "cascade_detector/cascade_detector.h"
#include "reefbot_msgs/FindSpecies.h"
#include "cascade_detector/DetectObject.h"
#include "reefbot_msgs/SingleSpeciesId.h"
#include "reefbot_msgs/SpeciesScore.h"

using namespace std;
using namespace ros;
using namespace cascade_detector;
using namespace reefbot_msgs;

class HandleServiceRequest {
public:
  HandleServiceRequest(CascadeDetector* client, int species_id)
    : client_(client), species_id_(species_id) {}
  bool operator()(reefbot_msgs::FindSpecies::Request& request,
                  reefbot_msgs::FindSpecies::Response& response) {
    ROS_DEBUG("Processing request");
    response.response.image_id = request.image.image_id;

    DetectObject detectObject;
    detectObject.request.image = request.image.image;
    if (client_->HandleServiceRequest(detectObject.request,
                                      detectObject.response)) {
      for (vector<cascade_detector::Detection>::const_iterator resultI =
             detectObject.response.detections.detections.begin();
           resultI != detectObject.response.detections.detections.end();
           resultI++) {
        SpeciesScore curScore;
        curScore.species_id = species_id_;
        curScore.score = resultI->score;
        curScore.meta_data = resultI->label;
        SingleSpeciesId curAnswer;
        curAnswer.bounding_box = resultI->mask.roi;
        curAnswer.best_species.push_back(curScore);

        response.response.answers.push_back(curAnswer);
      }
    } else {
      return false;
    }

    response.response.header.stamp = ros::Time::now();

    return true;

  };

private:
  CascadeDetector* client_;
  int species_id_;
  
};                          

int main(int argc, char** argv) {
  ros::init(argc, argv, "ReefbotCascadeDetector");

  NodeHandle handle;

  // Get the node parameters
  string filename;
  string objectName;
  string imageTopic;
  string responseTopic;
  string serviceName;
  int species_id;
  NodeHandle local("~");
  local.getParam("detector_filename", filename);
  local.param<string>("object_name", objectName, "face");
  local.param<string>("request_topic", imageTopic, "detect_object_request");
  local.param<string>("response_topic", responseTopic, "object_detection");
  local.param<string>("service_name", serviceName, "detect_object");
  local.param<int>("species_id", species_id, -1);

  string internalCVService = local.getNamespace()+'/'+serviceName;
  string internalCVRequest = local.getNamespace()+"/detect_object_request";
  string internalCVResponse = local.getNamespace()+"/object_detection";

  // Create the component that will do the open cv call
  cascade_detector::CascadeDetector node(filename, objectName);
  node.Init(internalCVRequest,
            internalCVResponse,
            internalCVService);

  // Now create a service handler that will provde the wrapper for Reefbot
  ROS_INFO_STREAM("Advertising service: " << serviceName);
  ServiceServer service = handle.advertiseService<
  reefbot_msgs::FindSpecies::Request,
  reefbot_msgs::FindSpecies::Response>(
    serviceName,
    HandleServiceRequest(&node, species_id));
                                                  
  
  ros::spin();
}
  
