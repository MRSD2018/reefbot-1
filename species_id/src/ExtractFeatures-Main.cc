// Program that can extract features from an image and dump them to a file
//
// Author: Mark Desnoyer (markd@cmu.edu)
// Date: June 2010

#include <boost/filesystem.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <gflags/gflags.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <string>

#include "cv_blobs/BlobResult-Inline.h"
#include "cv_blobs/Blob.h"
#include "ImageDescriptor-Inl.h"
#include "ImageDescriptorGenerator-Inl.h"
#include "SIFTDescriptorGenerator.h"
#include "ImageDescriptorGeneratorFlags.h"

using namespace species_id;
using namespace std;
using namespace boost;
using namespace cv;
using namespace cv_blobs;
namespace fs = boost::filesystem;

// -------- Define the command line options  ------------

// The input and output locations
DEFINE_string(input, "", "Input image file to get the features of");
DEFINE_string(output, "","File to output the image descriptor to. "
              "If this is a "
              "blob image, multiple files will be created suffixed by "
              "indicies.");

int main(int argc, char *argv[]) {
  google::SetUsageMessage("Program that extracts features from an image "
                          "and prints them in ascii format to a file.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Figure out the keypoint detector and descriptors to use
  VerifyDescriptorFlags();
  scoped_ptr<FeatureDetector> keyGenerator(ChooseFeatureDetector());
  scoped_ptr<ImageDescriptorGenerator<float> > descGenerator(
    ChooseImageDescriptor(keyGenerator.get()));
  ImageDescriptorINRIASerializer<float> serializer;

  // Now open up the input image
  ROS_INFO_STREAM("Extracting descriptors from " << FLAGS_input);
  vector<ImageRegion> masks;
  string imageFn;
  bool isBlobFile = FLAGS_input.rfind(".blob") != string::npos;
  if (isBlobFile) {
    // The file is a blob serialization
    BlobResultSerializer<int> blobReader;
    fstream blobStream(FLAGS_input.c_str(), ios::in);
    shared_ptr<BlobResult<int> > blobs =
      blobReader.Deserialize(blobStream, &imageFn,
                             fs::path(FLAGS_input).parent_path().string());

    for (int i = 0; i < blobs->nBlobs(); ++i) {
      masks.push_back(ImageRegion(*(blobs->GetBlob(i).ToImageRegion())));
    }
  } else {
    // The file is an actual image
    imageFn = FLAGS_input;
  }
    
  Mat image = imread(imageFn);
  if (image.empty()) {
    ROS_FATAL_STREAM("Could not read the image at file: " << imageFn);
  }

  // Next, create the descriptors of the image
  vector<shared_ptr<
    ImageDescriptorGenerator<float>::DescriptorCollection> >
    descriptors;
  descGenerator->ExtractUsingMasks(image, masks, &descriptors);

  // Finally, print out the descriptors
  for (unsigned int i = 0; i < descriptors.size(); ++i) {
    string outputFn(FLAGS_output);
    if (isBlobFile) {
      outputFn += '.' + lexical_cast<string>(i);
    } else {
      ROS_ASSERT(i == 0);
    }
    // Open the output file to write to
    fstream fileOut(outputFn.c_str(), ios::out | ios::trunc);
    ROS_INFO_STREAM("Writing descriptors to: " << outputFn);

    serializer.Write(fileOut, *(descriptors[i]));
  }

}


