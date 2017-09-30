// Routine that trains an integral cascade using a bag where candidate
// regions in images are scored. Those scored above a threshold and
// overlap a region are considered positive.
//
// Usage: TrainCascadeUsingBag [options] <bag> <detector_list>
// Author: Mark Desnoyer (mdesnoyer@gmail.com)
// Date: Jan 2013

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/unordered_set.hpp>
#include <gflags/gflags.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/scoped_ptr.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "hima_experiment/VisualUtilityEstimation.h"
#include "sensor_msgs/RegionOfInterest.h"

#include "hog_detector/integral_hog_cascade_trainer.h"
#include "hog_detector/hog_detector_internal.h"

// Settings to control how much data to use
DEFINE_int32(winH, 128, "Height of the cannoncial window");
DEFINE_int32(winW, 64, "Width of the cannoncial window");
DEFINE_int32(seed, 19843189, "Random seed to use");
DEFINE_int32(nsamples, 60000, "Number of samples of the images to learn on");

// Thresholds and costs. 
DEFINE_double(miss_cost, 0.5, "The cost of a miss");
DEFINE_double(false_pos_cost, 0.5, "The cost of a false positive "
              "hitting the high level HOG detector");
DEFINE_double(time_cost_per_error, 1.0,
              "The cost of 1 second of average processing per error");
DEFINE_double(score_thresh, 0.0,
              "Threshold for score to pass an example.");
DEFINE_double(overlap_thresh, 0.5,
              "Threshold for for the overlap to consider it a positive.");

// Input flags
DEFINE_string(integral_hist_time, "",
              "File containing a single line specifying the time it takes "
              "to copute an integral histogram");
DEFINE_string(fill_block_cache_time, "", 
              "File containing lines <blockW>,<blockH>,<time>");
DEFINE_string(svm_eval_time, "", "File containing lines "
              "<descriptorSize>,<subWinW>,<subWinH>,<time/window>");
DEFINE_string(true_hog_time, "", "File containing lines "
              "<nWindows>,<time>");

// The output flags
DEFINE_string(output, "integral_hog_cascade.xml",
              "Output filename for the trained cascade");

using namespace hog_detector;
using namespace hima_experiment;
using namespace cv;
using namespace std;
using namespace boost;

void ExtractFileList(const string& filename,
                      vector<string>* fileList) {
  ROS_ASSERT(fileList);

  char buf[512];

  ROS_INFO_STREAM("Opening " << filename << " to get file list");
  ifstream inStream(filename.c_str(), ios_base::in);
  while(!inStream.eof() && inStream.good()) {
    inStream.getline(buf, 512);
    if (!inStream.fail() && !inStream.bad()) {
      fileList->push_back(string(buf));
    }
  }

  ROS_INFO_STREAM("Found " << fileList->size() << " files");
  
}

int main(int argc, char** argv) {
  // Parse the input
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "integral_hog_trainer",
            ros::init_options::AnonymousName);

  if (argc < 3) {
    ROS_FATAL("Usage: TrainCascadeUsingImages [options] <detector_list> <bag0> <bag1> ... <bagN>");
    return 1;
  }
  
  // Create the trainer
  IntegralHogCascadeTrainer trainer(
    new IntegralCascadeTimeCalculator(FLAGS_integral_hist_time,
                                      FLAGS_fill_block_cache_time,
                                      FLAGS_svm_eval_time,
                                      FLAGS_true_hog_time),
    Size(FLAGS_winW, FLAGS_winH));

  // Get the list of bags to process
  vector<string> bagFiles;
  for (int i = 2; i < argc; ++i) {
    bagFiles.push_back(argv[i]);
  }

  // Figure out the number of images in the bag and the number of
  // regions per image so we know how many to sample.
  int nImages = 0;
  int nRegions = -1;
  BOOST_FOREACH(string bagFile, bagFiles) {
    // Open up the bag
    rosbag::Bag bag;
    bag.open(bagFile, rosbag::bagmode::Read);
    rosbag::View quickView(bag, rosbag::TopicQuery("results"));
    BOOST_FOREACH(rosbag::MessageInstance const msg, quickView) {
      nImages++;
      if (nRegions < 0) {
        VisualUtilityEstimation::ConstPtr data =
          msg.instantiate<VisualUtilityEstimation>();
        if (data == NULL) {
          ROS_ERROR_STREAM("Invalid message");
          continue;
        }
        nRegions = data->regions.size();
      }
    }
  }

  ROS_INFO_STREAM("Training on " << nImages 
                  << " images. Each image has " << nRegions
                  << " regions. We will use "
                  << FLAGS_nsamples / nImages << " regions per image");

  // Create the random number generator needed to choose the regions
  // we care about.
  mt19937 rng(FLAGS_seed);

  // Load up the detectors
  ROS_INFO("Adding all the candidate detectors");
  vector<string> detectorFiles;
  ExtractFileList(argv[1], &detectorFiles);
  for (vector<string>::const_iterator i = detectorFiles.begin();
       i != detectorFiles.end();
       ++i) {
    trainer.AddCandidateDetector(IntegralHogDetector::Ptr(
      new IntegralHogDetector(*i, Size(8,8))));
  }
  

  // Go through the bags and add examples to train on
  int curImgNum = 0;
  int totalNegs = 0;
  int totalNegsTrain = 0;
  BOOST_FOREACH(string bagFile, bagFiles) {
    ROS_INFO_STREAM("Loading images from " << bagFile);
    rosbag::Bag bag;
    bag.open(bagFile, rosbag::bagmode::Read);
    rosbag::View longView(bag, rosbag::TopicQuery("results"));
    BOOST_FOREACH(rosbag::MessageInstance const msg, longView) {
      curImgNum++;

      // Open the message
      VisualUtilityEstimation::ConstPtr data =
        msg.instantiate<VisualUtilityEstimation>();
      if (data == NULL) {
        continue;
      }

      // Add all the positive examples and store the locations of the
      // negative ones.
      vector<int> negIdx;
      vector<Rect> posEntries;
      for (unsigned int i = 0u; i < data->regions.size(); ++i) {
        if (data->scores[i] >= FLAGS_score_thresh &&
            data->overlaps[i] >= FLAGS_overlap_thresh) {
          const sensor_msgs::RegionOfInterest& region = data->regions[i];
          posEntries.push_back(Rect(region.x_offset,
                                    region.y_offset,
                                    region.width,
                                    region.height));
        } else {
          negIdx.push_back(i);
        }
      }
      int nPos = posEntries.size();
      
      // Choose which negative entries to train with
      unordered_set<int> chosenEntries;
      while (chosenEntries.size() < 
             (unsigned int)((FLAGS_nsamples - nPos) / nImages)) {
        chosenEntries.insert(uniform_int<int>(0, negIdx.size() - 1)(rng));
      }

      // Collect the negative regions to train with
      vector<Rect> negEntries;
      BOOST_FOREACH(int entry, chosenEntries) {
        const sensor_msgs::RegionOfInterest& region =
          data->regions[negIdx[entry]];
        negEntries.push_back(Rect(region.x_offset,
                                  region.y_offset,
                                  region.width,
                                  region.height));
      }
      

      ROS_INFO_STREAM("There are " << nPos
                      << " positive regions and " << negIdx.size()
                      << " negative regions. Training with "
                      << negEntries.size() << " negative regions.");
      totalNegs += negIdx.size();
      totalNegsTrain += negEntries.size();

      // Load up the image
      ROS_INFO_STREAM("Opening " << data->image);
      Mat image = imread(data->image);
      if (image.empty()) {
        ROS_ERROR_STREAM("Could not open image " << data->image
                         << " skipping.");
        continue;
      }

      // Add the entries for training
      trainer.AddImageForTraining(image, posEntries, 1.0);
      trainer.AddImageForTraining(image, negEntries, -1.0);
    }
  }

  ROS_INFO_STREAM("Starting to train");
  trainer.TrainSupSubWithLoadedData(FLAGS_miss_cost,
                                    FLAGS_false_pos_cost,
                                    FLAGS_time_cost_per_error,
                                    ((float)totalNegs) / totalNegsTrain);

  ROS_INFO_STREAM("Done training.");

  ROS_INFO_STREAM("Saving the cascade to: " << FLAGS_output);
  trainer.cascade()->save(FLAGS_output);
}
