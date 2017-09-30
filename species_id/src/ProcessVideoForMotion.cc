// Program that takes a video as input and processes each frame for
// motion. The output is a video with the motion highlighted in red
//
// Author: Mark Desnoyer markd@cmu.edu
// Date: June 2011

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gflags/gflags.h>
#include <iostream>
#include "MotionExtractor.h"
#include "cv_blobs/BlobResult-Inline.h"

DEFINE_string(input, "", "Input video file.");
DEFINE_string(output, "", "Output video file.");
DEFINE_double(diff_thresh, 800, "Threshold for the differencing to find motion.");
DEFINE_double(merge_thresh, 1, "Threshold for merging blobs.");
DEFINE_int32(max_warp_iter, 50, "Maximum number of iterations for warping a frame.");
DEFINE_double(min_warp_precision, 1e-5, "Minimum precision required for the stability of the transformation matrix in the warping");

using namespace species_id;
using namespace cv_blobs;
using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  MotionExtractor motionExtractor(FLAGS_diff_thresh, FLAGS_merge_thresh, FLAGS_max_warp_iter, FLAGS_min_warp_precision);

  // Open the file for input
  VideoCapture videoIn(FLAGS_input);
  if (!videoIn.isOpened()) {
    cerr << "Could not open " << FLAGS_input << endl;
    return 1;
  }

  // Open the file for output
  bool doFileOutput = FLAGS_output.size() > 0;
  VideoWriter videoOut;
  if (!doFileOutput) {
    namedWindow("motion", 1);
  }
  Mat motionFrame;
  int time = 0;
  while (videoIn.grab()) {
    Mat rawFrame;
    videoIn.retrieve(rawFrame, 0);

    Mat smallFrame;
    resize(rawFrame, smallFrame, Size(), 0.5, 0.5);

    motionExtractor.AddImage(smallFrame, time);
    if (time - motionExtractor.time() < 1.0) {
      // We were able to calculate the motion, so process the image
      Mat objectImage = motionExtractor.RetrieveObjects().ToImage();
      objectImage = objectImage > 0;
      resize(objectImage, objectImage, Size(), 2.0, 2.0);

      // Now merge the object image into the red channel of the current frame
      vector<Mat> channels;
      split(rawFrame, channels);
      channels[2] = channels[2] / 2 + 128 * objectImage;
      merge(channels, motionFrame);

      if (doFileOutput) {
        if (!videoOut.isOpened()) {
          if (!videoOut.open(FLAGS_output, CV_FOURCC_DEFAULT, 30.0, motionFrame.size(), true)) {
            cerr << "Could not open file for output: " << FLAGS_output << endl;
            return 3;
          }
        }
        videoOut << motionFrame;
      } else {
        imshow("motion", motionFrame);
      }
    }

    time += 1;

    if (time % 10 == 0) {
      cout << "Processed " << time << " frames." << endl;
    }

  } 

}
