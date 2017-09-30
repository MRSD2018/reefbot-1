// Program that builds a new ImageSearchIndex for the species ids.
//
// Takes as input, a CSV file where each line is:
//
// <blob_filename>,<blob_id>,<label>
//
// Author: Mark Desnoyer markd@cmu.edu
// Date: Nov 2010

#include "base/BasicTypes.h"
#include "ros/ros.h" // For logging and parameters
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <gflags/gflags.h>
#include <fstream>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>

#include "cv_blobs/Blob.h"
#include "SpeciesIDNode.h"
#include "cv_bridge/CvBridge.h"
#include "reefbot_msgs/AddSpeciesLabel.h"
#include "reefbot_msgs/ImageRegion.h"

#include "cv_blobs/BlobResult-Inline.h"

DEFINE_string(input, "", "Filename of the input csv");
DEFINE_string(output, "", "Filename for the output ImageSearchIndex");
DEFINE_string(blob_prefix, "", "Prefix to add to the blob filename");
DEFINE_string(color_dict_filename, "", "Filename of the color dictionary");
DEFINE_string(color_converter, "CV_BGR2HSV", "String like CV_BGR2HSV that defines the color conversion.");
DEFINE_double(color_frac, 0.1, "Fraction of pixels to use in the color histogram");
DEFINE_string(shape_dict_filename, "", "Filename for the shape dictionary");
DEFINE_bool(surf_detector, false, "Use the surf detector?");
DEFINE_double(surf_hessian_threshold, 400, "Surf hessian threshold");
DEFINE_int32(surf_octaves, 3, "Number of octaves used in surf");
DEFINE_int32(surf_octave_layers, 4, "Number of octave layers in SURF");
DEFINE_bool(surf_extended, false, "Use the extended, bigger SURF descriptor");
DEFINE_bool(surf_descriptor, false, "Do we use the surf descriptor");
DEFINE_bool(sift_descriptor, false, "Do we use the SIFT descriptor");
DEFINE_double(shape_weight, 0.5, "Weight to give to the shape values compared to color");
DEFINE_double(min_shape_val, 1e-2, "Minimum value to include the shape.");
DEFINE_double(min_color_val,  1e-2, "Minimum value of a color level to include in the descriptor.");
DEFINE_double(min_score, 1e-2, "Minimum matching score to allow the value to be returned");
DEFINE_bool(opponent_color_surf, false, "Do we use the opponent color SURF descriptor?");
DEFINE_bool(cinvariant_color_surf, false, "Do we use the C-SURF descriptor?");

DEFINE_bool(use_bounding_box, false, "Instead of the tight blob, use the bounding box?");
DEFINE_double(bounding_box_expansion, 1.0, "If use_bounding_box, then the fraction to grow the bounding box");
DEFINE_bool(geometric_rerank, false, "Do you want geometric reranking");
DEFINE_double(geo_rerank_inlier_thresh, 3.0, "The threshold for inliers when geometric reranking");

inline int max(int x, int y){ return x<y ? y : x; }
inline int min(int x, int y){ return x<y ? x : y; }

using namespace species_id;
using namespace boost;
using namespace std;
using reefbot_msgs::ImageRegion;
using namespace cv_blobs;

struct BlobLabel {
  BlobLabel(uint32 _blobId, uint32 _label)
    : blobId(_blobId), label(_label) {}

  uint32 blobId;
  uint32 label;
};

void ProcessBlobLabels(const string& blobFilename,
                       const vector<BlobLabel>& blobLabels,
                       SpeciesIDNode& node) {
  if (blobFilename.size() == 0) {
    return;
  }

  // Open up the blobs description
  ifstream blobFile((FLAGS_blob_prefix + blobFilename).c_str());
  if (!blobFile.is_open()) {
    ROS_ERROR_STREAM("Error opening file: "
                     << (FLAGS_blob_prefix + blobFilename));
    return;
  }

  // Read in the blobs
  BlobResultSerializer<uint8> blobReader;
  string imgFilename;
  shared_ptr<BlobResult<uint8> > blobs = blobReader.Deserialize(
    blobFile,
    &imgFilename,
    FLAGS_blob_prefix);

  // Read in the image
  Mat cvImage = imread(imgFilename, -1);

  // Make sure that the image is a color image
  if (cvImage.empty()) {
    ROS_ERROR_STREAM("Could not open image: " << imgFilename);
    return;
  }
  if (cvImage.channels() != 3) {
    ROS_ERROR_STREAM("Image " << imgFilename
                     << " was not a 3 color image. It has "
                     << cvImage.channels() << " channels");
    return;
  }
  if (cvImage.depth() != CV_8U && cvImage.depth() != CV_8S) {
    ROS_ERROR_STREAM("Image " << imgFilename
                     << " was not an 8-bit image. It has "
                     << cvImage.depth() << " bit depth");
    return;
  }

  // Create the ROS message
  reefbot_msgs::AddSpeciesLabel::Ptr msg(new reefbot_msgs::AddSpeciesLabel());
  IplImage iplImage = IplImage(cvImage);
  // For some reason, opencv puts images in BGR format, so label that
  sensor_msgs::Image::Ptr rosImg =
    sensor_msgs::CvBridge().cvToImgMsg(&iplImage, "bgr8");
  msg->image = *rosImg;
  msg->filename = blobFilename;

  vector<shared_ptr<ImageRegion> > imageRegions;
  
  for (vector<BlobLabel>::const_iterator blobI = blobLabels.begin();
       blobI != blobLabels.end();
       ++blobI) {
    msg->labels.push_back(blobI->label);
    msg->blob_ids.push_back(blobI->blobId);

    // Store the shared pointer for the image region in a data
    // structure that will last as long as msg does
    shared_ptr<ImageRegion> region =
      blobs->GetBlob(blobI->blobId).ToImageRegion();

    if (FLAGS_use_bounding_box) {
      // Only use the bounding box and change its size as necessary
      region->mask = sensor_msgs::Image();
      
      int centerX = region->bounding_box.x_offset + 
        region->bounding_box.width/2;
      int centerY = region->bounding_box.y_offset + 
        region->bounding_box.height/2;
      int minX = max(
        0, 
        centerX - region->bounding_box.width*FLAGS_bounding_box_expansion/2);
      int minY = max(
        0, 
        centerY - region->bounding_box.height*FLAGS_bounding_box_expansion/2);
      int maxX = min(
        cvImage.cols, 
        centerX + region->bounding_box.width*FLAGS_bounding_box_expansion/2);
      int maxY = min(
        cvImage.rows, 
        centerY + region->bounding_box.height*FLAGS_bounding_box_expansion/2);

      region->bounding_box.x_offset = minX;
      region->bounding_box.y_offset = maxY;
      region->bounding_box.width = maxX - minX;
      region->bounding_box.height = maxY - minY;
    }


    imageRegions.push_back(region);
    msg->regions.push_back(*region);
  }

  // Now add all the blobs in this image to the index
  node.HandleAddLabel(msg);
  
}


int main(int argc, char** argv) {
  // Parse the input
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Open up a SpeciesIDNode which we'll use to accumulate examples
  // for the index.
  //ros::init(argc, argv, "BuildFishIndex");
  SpeciesIDNode node;

  node.Init(FLAGS_color_dict_filename,
            FLAGS_color_converter,
            FLAGS_color_frac,
            FLAGS_shape_dict_filename,
            FLAGS_surf_detector,
            FLAGS_surf_hessian_threshold,
            FLAGS_surf_octaves,
            FLAGS_surf_octave_layers,
            FLAGS_surf_descriptor,
            FLAGS_surf_extended,
            FLAGS_sift_descriptor,
            FLAGS_shape_weight,
            FLAGS_min_shape_val, 
            FLAGS_min_color_val,
            FLAGS_min_score,
            FLAGS_opponent_color_surf,
            FLAGS_cinvariant_color_surf,
            FLAGS_geometric_rerank,
            FLAGS_geo_rerank_inlier_thresh);

  // Open up the csv file
  ifstream input(FLAGS_input.c_str());
  if (!input.is_open()) {
    ROS_ERROR_STREAM("Error opening file: " << FLAGS_input);
    return 3;
  }

  // Parse each line
  string line;
  vector<string> splitLine;
  string curFilename = "";
  vector<BlobLabel> blobLabels;
  while (getline(input, line)) {
    // Use a tokenizer to split the line
    tokenizer<escaped_list_separator<char> > tok(line);
    splitLine.assign(tok.begin(), tok.end());

    if (splitLine.size() != 3) {
      ROS_ERROR_STREAM("Wrong number of fields in line: " << line);
      continue;
    }

    // See if this is a new image we have data for
    if (splitLine[0].compare(curFilename) != 0) {
      ProcessBlobLabels(curFilename, blobLabels, node);
      blobLabels.clear();
    }

    // Collect all the labels for this image
    curFilename = splitLine[0];
    try {
      BlobLabel blobLabel(lexical_cast<uint32>(splitLine[1]),
                          lexical_cast<uint32>(splitLine[2]));
      blobLabels.push_back(blobLabel);
    } catch (bad_lexical_cast&) {
      ROS_ERROR_STREAM("Could not parse numbers in line: " << line);
      continue;
    }
  }

  node.RebuildIndex();
  node.SaveIndexToFile(FLAGS_output);

  return 0;
}
