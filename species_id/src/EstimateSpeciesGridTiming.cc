// Program that estimates the timing to classifier entries on a grid.
//
// Assumes that windows will be included based on a long tail
// distribution, so windows closer to previously selected windows are
// more likely to be chosen next.
//
// Output file is of the form:
//
// <nWindows>,<seconds>
// 
// Usage: EstimateSpeciesGridTiming [options] <indexFile> <outputFile> <sampleImage1> ... <sampleImageN>

#include <gflags/gflags.h>
#include <fstream>
#include <boost/scoped_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <boost/random/mersenne_twister.hpp>
#include "SpeciesIDNode.h"

DEFINE_int32(samples, 100,
             "Number of samples to take along the number of possible entries");
DEFINE_int32(seed, 165494, "Random number generator seed");
DEFINE_int32(minW, 32, "Minimum width");
DEFINE_int32(minH, 32, "Minimum height");
DEFINE_int32(winStride, 16, "X and Y stride");
DEFINE_double(scale_stride, 1.20, "Width and height stride");

using namespace std;
using namespace boost;
using namespace boost::accumulators;
using namespace cv;
using namespace species_id;

// Create a mask of the valid spots in the grid
//
// Returns the number of valid entries
int CreateFullMask(const Size& imageSize, Mat_<bool>* mask) {
  vector<double> widths;
  for (double curW = FLAGS_minW; curW < imageSize.width;
       curW *= FLAGS_scale_stride) {
    widths.push_back(std::round(curW));
  }
  vector<double> heights;
  for (double curH = FLAGS_minH; curH < imageSize.height;
       curH *= FLAGS_scale_stride) {
    heights.push_back(std::round(curH));
  }

  vector<int> maskSize;
  maskSize.push_back(widths.size());
  maskSize.push_back(heights.size());
  maskSize.push_back(std::max<int>(1,
    (imageSize.width - FLAGS_minW) / FLAGS_winStride));
  maskSize.push_back(std::max<int>(1,
    (imageSize.height - FLAGS_minH) / FLAGS_winStride));
  *mask = Mat_<bool>::zeros(4, &maskSizes[0]);

  int nValid = 0;
  for (unsigned int widthI = 0u; widthI < widths.size(); ++widthI) {
    for (unsigned int heightI = 0u; heightI < heights.size(); ++heightI) {
      for (int i = 0, curX = 0; curX < imageSize.width-widths[widthI]; ++i) {
        for (int j = 0, curY = 0; curY < imageSize.height-heights[heightI];
             ++j) {
          (*mask)(widthI, heightI, i, j) = true;
          nValid++;
          curY += FLAGS_winStride;
        }
        curX += FLAGS_winStride;
      }
    }
  }

  return nValid;
}

Mat_<bool> createRandomMask(const Mat_<bool>& fullMask, int nWindows,
                            mt19937& randGen) {
  Mat_<bool> retval;

  // We select which windows to process using a gaussian process that
  // assumes regions close to previously selected regions are more
  // likely to be chosen.
  Mat_<double> pChoose(fullMask);

  if (nWindows == countNonZero(fullMask)) {
    return Mat_<bool>(fullMask);
  }

  for (int nChosen = 0; nChosen < nWindows; ++nChosen) {
    pChoose /= sum(pChoose);
    
  }
}


int main(int argc, char **argv) {

  google::ParseCommandLineFlags(&argc, &argv, true);

  string indexFile(argv[1]);
  string outputFile(argv[2]);
 
  // Grab the list of images
  vector<string> imageFiles;
  for (int i = 3; i < argc; ++i) {
    imageFiles.push_back(string(argv[i]));
  } 

  // Create the species id node to use
  SpeciesIDNode node;
  node.LoadIndexFromFile(indexFile);

  // Create accumulators for each count of windows.
  vector<accumulator_set<double, stats<tag::variance> >
    timeAccumulators(FLAGS_samples + 1);

  // Intialize the random number generator
  mt19937 randGen(FLAGS_seed);

  for (vector<string>::const_iterator fileI = imageFiles.begin();
       fileI != imageFiles.end(); ++fileI) {

    Mat image = imread(*fileI);

    Mat_<bool> fullMask;
    int maxWindows = CreateFullMask(image.size(), &fullMask);

    for (int i = 0; i <= FLAGS_samples; ++i) {
      int nWindows = std::round<int>(((double)maxWindows) * i /
                                     FLAGS_samples);

      Mat_<bool> randomMask = CreateRandomMask(fullMask, nWindows, randGen);

      double processingTime = GetTimeForMaskedRegions(image, randomMask);
      timeAccumulators[i](processingTime);
    }
  }
 

}
