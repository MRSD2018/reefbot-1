// ROS Node that loads up a search index and then acts as a ROS Node
// that handles SpeciesIDRequests.
//
// ROS Input Type: SpeciesIDRequest
// ROS Output Type: SpeciesIDResponse
//
// Author: Mark Desnoyer markd@cmu.edu
// Date: Oct 2010

#include "ros/ros.h"
#include "SpeciesIDNode.h"

using namespace std;
using namespace ros;

int main(int argc, char** argv) {
  ros::init(argc, argv, "SpeciesIDNode");

  NodeHandle handle;
  NodeHandle local("~");

  // Get all the parameters for the node
  string colorDictFilename;
  string colorConverterStr;
  double colorFrac;
  string shapeDictFilename;
  bool useSurfDetector;
  double hessianThreshold;
  int octaves;
  int octaveLayers;
  bool useSurfDescriptor;
  bool useSiftDescriptor;
  bool extendedSurf;
  double shapeWeight;
  double minShapeVal;
  double minColorVal;
  double minScore;
  bool useOpponentColorSurf;
  bool useCInvariantSurf;
  bool doGeometricRerank;
  double geoRerankInlierThresh;

  local.getParam("color_dict_filename", colorDictFilename);
  local.param<string>("color_converter", colorConverterStr, "CV_BGR2HSV");
  local.param("color_frac", colorFrac, 0.1);
  local.getParam("shape_dict_filename", shapeDictFilename);
  local.param("surf_detector", useSurfDetector, false);
  local.param<double>("surf_hessian_threshold", hessianThreshold, 400);
  local.param("surf_octaves", octaves, 3);
  local.param("surf_octave_layers", octaveLayers, 4);
  local.param("surf_extended", extendedSurf, false);
  local.param("surf_descriptor", useSurfDescriptor, false);
  local.param("sift_descriptor", useSiftDescriptor, false);
  local.param("shape_weight", shapeWeight, 0.5);
  local.param("min_shape_val", minShapeVal, 1e-2);
  local.param("min_color_val", minColorVal, 1e-2);
  local.param("min_score", minScore, 1e-2);
  local.param("opponent_color_surf", useOpponentColorSurf, false);
  local.param("cinvariant_color_surf", useCInvariantSurf, false);
  //local.param("geometric_rerank", doGeometricRerank, false);
  //local.param("geo_rerank_inlier_thresh", geoRerankInlierThresh, 3.0);
  local.getParam("geometric_rerank", doGeometricRerank);
  local.getParam("geo_rerank_inlier_thresh", geoRerankInlierThresh);
  
  species_id::SpeciesIDNode node;

  string indexFile;

  if (local.getParam("index_file", indexFile)) {
    ROS_WARN_STREAM("Loading parameters from index file. NOT command line");
    node.LoadIndexFromFile(indexFile);
  } else {
    node.Init(colorDictFilename,
              colorConverterStr,
              colorFrac,
              shapeDictFilename,
              useSurfDetector,
              hessianThreshold,
              octaves,
              octaveLayers,
              useSurfDescriptor,
              extendedSurf,
              useSiftDescriptor,
              shapeWeight,
              minShapeVal,
              minColorVal,
              minScore,
              useOpponentColorSurf,
              useCInvariantSurf,
              doGeometricRerank,
              geoRerankInlierThresh);
  }

  node.ConnectToROS(handle);

  node.Spin();
}
