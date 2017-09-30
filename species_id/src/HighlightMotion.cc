// Program that processes a video file by highlighting the objects
// identified from motion in red.
//
// Usage: highligh_motion [options]
//
// Author: Mark Desnoyer (mdesnoyer@gmail.com)
// Date: July 2011

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gflags/gflags.h>
#include <ros/ros.h> // For logging
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include <fstream>
#include <iomanip>

#include "MotionExtractor.h"
#include "visual_utility/TransformEstimator.h"
#include "EvolutionModel.h"
#include "cv_blobs/BlobResult-Inline.h"
#include "visual_utility/cvutils.h"

DEFINE_string(input, "", "Filename of the input movie");
DEFINE_string(output_dir, "", "Directory for the outputs");
DEFINE_bool(do_video_output, false, "Should we output video?");
DEFINE_string(output_video, "", "Filename for the output movie");
DEFINE_bool(do_blob_output, false, "Should we output the blob files and frames?");
DEFINE_string(blob_prefix, "blob-", "Prefix for the blob filename");
DEFINE_string(frame_prefix, "frame-", "Prefix for the frame filename");
DEFINE_double(blob_rate, 1.0, "Maximum rate in Hz to output blobs and frames");
DEFINE_int32(affine_iter, 100,
             "Maximum number of iterations for the affine estimation");
DEFINE_double(affine_precision, 1e-7,
              "Minimum precision for the affine estimation");
DEFINE_double(affine_scaling, 4.0,
              "Factor to shrink the image when finding the affine transform");
DEFINE_double(gauss_sigma, 10.0,
              "Size of the gaussian sigma for the evolution model");
DEFINE_double(gauss_alpha, 0.1, "Mixing parameter for gaussian vs. uniform "
              "distrubtion for the evolution model. Larger biases to the "
              "uniform distribution");
DEFINE_double(confidence_thresh, 0.5,
              "The probability threshold for identifying an object");
DEFINE_double(pareto_thresh, 0.03,
              "Assuming a pareto distribution, the fraction of pixels that "
              "are objects in a given frame");
DEFINE_double(dist_decay, 2.0,
              "The sharpness of the sigmoid function.");
DEFINE_int64(min_obj_size, 150L,
              "The minimum size in pixels for an object");
DEFINE_double(prior_alpha, 0.6,
              "Mixing parameter for the prior vs. the current distance. "
              "Larger values give prefernce to the current distance.");
DEFINE_double(fps, 30.0,
              "The output frame rate.");
DEFINE_bool(flip_colors, false,
            "Set to true to flip from RGB to BGR on the input stream.");

using namespace species_id;
using namespace std;
using namespace cv;
using namespace boost;
using namespace boost::filesystem;
using namespace cv_blobs;

// Abstract class for an output stream
class OutputGenerator {
 public:
  OutputGenerator(): curFrame_(0) {}

  virtual bool Init(VideoCapture* inputStream)=0;

  virtual void ProcessFrame(const Mat& frame,
                            const BlobResult<float>& objects,
                            const MotionExtractor& motionExtractor,
                            double curTime)=0;
  void IncrementFrame() { curFrame_++; }

protected:
  int curFrame_;
};

// Object to output a video with the motion highlighted in red
class MotionVideoOutput : public OutputGenerator{
 public:
  MotionVideoOutput() {}

  virtual bool Init(VideoCapture* inputStream) {
    if (!outputStream_.open(
      FLAGS_output_dir + "/" + FLAGS_output_video,
      CV_FOURCC('M', 'P', 'G', '1'),
      FLAGS_fps,
      Size2i(static_cast<int>(inputStream->get(CV_CAP_PROP_FRAME_WIDTH)),
             static_cast<int>(inputStream->get(CV_CAP_PROP_FRAME_HEIGHT))),
      true //isColor
      )) {
      ROS_FATAL_STREAM("Error opening the output video: "
                       << FLAGS_output_dir + "/" + FLAGS_output_video);
      return false;
    }
    return true;
  }

  virtual void ProcessFrame(const Mat& frame,
                            const BlobResult<float>& objects,
                            const MotionExtractor& motionExtractor,
                            double curTime) {
    // Creae a new frame showing the objects in red
    vector<Mat> channels;
    split(frame, channels);
    channels[2] = 5*channels[2]/10 + 0.5 * objects.ToBinaryImage();
    merge(channels, outputFrame_);

    // Draw a rectangle around the highest point
    rectangle(outputFrame_,
              motionExtractor.maxLocation() - Point(10,10),
              motionExtractor.maxLocation() + Point(10,10),
              CV_RGB(255,215,0),
              2);

    outputStream_ << outputFrame_;
  }

 private:
  VideoWriter outputStream_;
  Mat outputFrame_;
};

class BlobOutput : public OutputGenerator {
 public:
  BlobOutput() :
    blobDir_(), frameDir_(), lastTime_(0.0) {}

  virtual bool Init(VideoCapture* inputStream) {
    blobDir_ = path(FLAGS_output_dir) / "blobs";
    frameDir_ = path(FLAGS_output_dir) / "frames";

    create_directory(blobDir_);
    create_directory(frameDir_);

    return true;
  }

  virtual void ProcessFrame(const Mat& frame,
                            const BlobResult<float>& objects,
                            const MotionExtractor& motionExtractor,
                            double curTime) {
    if ((curTime - lastTime_) < 1.0 / FLAGS_blob_rate) {
      // We dont' want blobs for every frame silly
      return;
    }

    stringstream frameNumber;
    frameNumber << setw(7) << setfill('0') << curFrame_;

    // First output the frame
    path frameFilename =  FLAGS_frame_prefix + frameNumber.str() + ".jpg";
    imwrite((frameDir_ / frameFilename).string(), frame);

    // Now output the blob file
    path blobFilename = FLAGS_blob_prefix + frameNumber.str() + ".blob";
    ofstream outStream((blobDir_ / blobFilename).string().c_str());
    serializer_.Serialize(outStream, objects,
                          string("../frames/") + frameFilename.string());
    outStream.close();

    lastTime_ = curTime;
  }

 private:
  path blobDir_;
  path frameDir_;
  double lastTime_;

  BlobResultSerializer<float> serializer_;
};


int main(int argc, char** argv) {
  // Parse the input
  google::ParseCommandLineFlags(&argc, &argv, true);

  visual_utility::AffineTransformEstimator transformEstimator(
    FLAGS_affine_iter,
    FLAGS_affine_precision,
    FLAGS_affine_scaling);

  EvolutionModel* evolutionModel = new GaussianEvolutionModel(
    FLAGS_gauss_sigma,
    FLAGS_gauss_sigma,
    FLAGS_gauss_alpha);

  MotionExtractor motionExtractor(transformEstimator,
                                  evolutionModel,
                                  FLAGS_confidence_thresh,
                                  FLAGS_pareto_thresh,
                                  FLAGS_dist_decay,
                                  FLAGS_min_obj_size,
                                  FLAGS_prior_alpha);

  // Open up the input video
  VideoCapture inputStream;
  if (!inputStream.open(FLAGS_input)) {
    ROS_FATAL_STREAM("Error opening the input video: " << FLAGS_input);
    return 1;
  }

  // Create the output directory
  create_directory(FLAGS_output_dir);

  // Setup the outputs
  vector<shared_ptr<OutputGenerator> > outputs;
  if (FLAGS_do_video_output) {
    outputs.push_back(shared_ptr<OutputGenerator>(new MotionVideoOutput()));
  }
  if (FLAGS_do_blob_output) {
    outputs.push_back(shared_ptr<OutputGenerator>(new BlobOutput()));
  }

  if (outputs.empty()) {
    ROS_FATAL_STREAM("Must select an output type");
    exit(1);
  }

  for (unsigned int i = 0u; i < outputs.size(); ++i) {
    outputs[i]->Init(&inputStream);
  }

  double curTime = 0;
  Mat curFrame;
  Mat outputFrame;
  while (inputStream.grab()) {
    if (!inputStream.retrieve(curFrame, 0)) {
      ROS_ERROR_STREAM("Error retreiving the frame");
      break;
    }
    if (FLAGS_flip_colors) {
      cvtColor(curFrame, curFrame, CV_RGB2BGR);
    }

    motionExtractor.AddImage(curFrame, curTime);

    const BlobResult<float>& objects = motionExtractor.RetrieveObjects();
    if (objects.nBlobs() > 0 && 
        (curTime - motionExtractor.time() < 5.0/FLAGS_fps)){
      for (unsigned int i = 0u; i < outputs.size(); ++i) {
        outputs[i]->ProcessFrame(curFrame, objects, motionExtractor, curTime);
      }

    }      

    ROS_INFO_STREAM("Processed " << curTime << " seconds.");
    curTime += 1./FLAGS_fps;
    for (unsigned int i = 0u; i < outputs.size(); ++i) {
      outputs[i]->IncrementFrame();
    }
  }
                                  
  return 0;
}
