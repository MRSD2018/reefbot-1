/// ROS Node that loads up a search index and then acts as a ROS Node
// that handles SpeciesIDRequests (so serves responses) and/or handles
// AddSpeciesIDLabel to build up an index.
//
// ROS Input Types:
//   SpeciesIDRequest
//   AddSpeciesIDLabel
// ROS Output Type:
//   SpeciesIDResponse
//
// Author: Mark Desnoyer markd@cmu.edu
// Date: Oct 2010

#ifndef __SPECIES_ID_SPECIES_ID_NODE_H__
#define __SPECIES_ID_SPECIES_ID_NODE_H__

#include "ros/ros.h"
#include <boost/scoped_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/shared_ptr.hpp>
#include <ostream>
#include "opencv2/features2d/features2d.hpp"
#include "base/StringPiece.h"
#include "reefbot_msgs/SpeciesIDRequest.h"
#include "reefbot_msgs/AddSpeciesLabel.h"
#include "species_id/SpeciesID.h"
#include "species_id/SpeciesIDGrid.h"
#include "cv_bridge/cv_bridge.h"

#include "ColorDescriptorGenerator.h"
#include "DocSimilarityCalculator.h"
#include "HistogramDescriptorGenerator.h"
#include "MultiHistogramDocGenerator.h"
#include "ImageSearchIndex.h"
#include "RandomDetector.h"

namespace species_id {

class SpeciesDescriptorParameters {
public:
  SpeciesDescriptorParameters() : version_(-1) {}
  SpeciesDescriptorParameters(const StringPiece& colorDictFilename_,
                              const StringPiece& colorConverterStr_,
                              double colorFrac_,
                              const StringPiece& shapeDictFilename_,
                              bool useSurfDetector_,
                              double hessianThreshold_,
                              int octaves_,
                              int octaveLayers_,
                              bool useSurfDescriptor_,
                              bool extendedSurf_,
                              bool useSiftDescriptor_,
                              double shapeWeight_,
                              double minShapeVal_,
                              double minColorVal_,
                              double minScore_,
                              bool useOpponentColorSurf_,
                              bool useCInvariantSurf_,
                              bool doGeometricRerank_,
                              double geoRerankInlierThresh_)
    : version_(3), colorDictFilename(colorDictFilename_.as_string()), colorConverterStr(colorConverterStr_.as_string()), colorFrac(colorFrac_), shapeDictFilename(shapeDictFilename_.as_string()), useSurfDetector(useSurfDetector_), hessianThreshold(hessianThreshold_), octaves(octaves_), octaveLayers(octaveLayers_), useSurfDescriptor(useSurfDescriptor_), extendedSurf(extendedSurf_), useSiftDescriptor(useSiftDescriptor_), shapeWeight(shapeWeight_), minShapeVal(minShapeVal_), minColorVal(minColorVal_), minScore(minScore_),  useOpponentColorSurf(useOpponentColorSurf_), useCInvariantSurf(useCInvariantSurf_), doGeometricRerank(doGeometricRerank_), geoRerankInlierThresh(geoRerankInlierThresh_) {}

private:
  int32 version_;

public:
  int32 version() const { return version_; }
  string colorDictFilename;
  string colorConverterStr;
  double colorFrac;
  string shapeDictFilename;
  bool useSurfDetector;
  double hessianThreshold;
  int32 octaves;
  int32 octaveLayers;
  bool useSurfDescriptor;
  bool extendedSurf;
  bool useSiftDescriptor;
  double shapeWeight;
  double minShapeVal;
  double minColorVal;
  double minScore;

  // Version 2
  bool useOpponentColorSurf;
  bool useCInvariantSurf;

  // Version 3
  bool doGeometricRerank;
  double geoRerankInlierThresh;

private:  
  // Routines to pickle this object.
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(version_);
    ar & BOOST_SERIALIZATION_NVP(colorDictFilename);
    ar & BOOST_SERIALIZATION_NVP(colorConverterStr);
    ar & BOOST_SERIALIZATION_NVP(colorFrac);
    ar & BOOST_SERIALIZATION_NVP(shapeDictFilename);
    ar & BOOST_SERIALIZATION_NVP(useSurfDetector);
    ar & BOOST_SERIALIZATION_NVP(hessianThreshold);
    ar & BOOST_SERIALIZATION_NVP(octaves);
    ar & BOOST_SERIALIZATION_NVP(octaveLayers);
    ar & BOOST_SERIALIZATION_NVP(useSurfDescriptor);
    ar & BOOST_SERIALIZATION_NVP(extendedSurf);
    ar & BOOST_SERIALIZATION_NVP(useSiftDescriptor);
    ar & BOOST_SERIALIZATION_NVP(shapeWeight);
    ar & BOOST_SERIALIZATION_NVP(minShapeVal);
    ar & BOOST_SERIALIZATION_NVP(minColorVal);
    ar & BOOST_SERIALIZATION_NVP(minScore);

    if (version >= 2) {
      ar & BOOST_SERIALIZATION_NVP(useOpponentColorSurf);
      ar & BOOST_SERIALIZATION_NVP(useCInvariantSurf);
    }

    if (version >= 3) {
      ar & BOOST_SERIALIZATION_NVP(doGeometricRerank);
      ar & BOOST_SERIALIZATION_NVP(geoRerankInlierThresh);
    }
  }
};

std::ostream& operator<< (std::ostream& out, const SpeciesDescriptorParameters& s);

class SpeciesIDNode {
public:
  // Constructor that creates the histogram generator based on the ROS params
  SpeciesIDNode();

  // Initialize the node with the necessary parameters
  //
  // Inputs:
  // colorDictFilename - filename of the color dictionary file
  // colorConverter - OpenCV color conversion string (e.g. CV_BGR2HSV)
  // colorFrac - Fraction of pixels in the image to use for the histogram
  //             of the colors.
  // shapeDictFilename - filename of the shape dictionary file
  // useSurfDetector - Use the surf detector to find keypoints
  // hessianThreshold - Parameter for SURF
  // octaves - Parameter for SURF
  // octaveLayers - Parameter for SURF
  // extendedSurf - Use the extended SURF descriptor
  // useSurfDescriptor - Use surf descriptor at each keypoint
  // useSiftDescriptor - Use the SIFT descriptor at each keypoint
  // shapeWeight - Fraction of the total score to shape. Color gets 1-this
  // minShapeVal - Minimum value to consider for a shape match. 
  //               Higher value will speed up the lookup, but reduce accuracy.
  // minColorVal - Minimum value to consider for a color match.
  // minScore - Minimum score to return a result at all
  // useOpponentColorSurf - Use the opponent color SURF descriptor
  // useCInvariantSurf - Use the c invariant SURF descriptor
  // doGeometricRerank - Rerank the images using geometric data 
  // geoRerankInlierThresh - Distance threshold to identify the inlier
  void Init(const StringPiece& colorDictFilename,
            const StringPiece& colorConverter,
            double colorFrac,
            const StringPiece& shapeDictFilename,
            bool useSurfDetector,
            double hessianThreshold,
            int octaves,
            int octaveLayers,
            bool useSurfDescriptor,
            bool extendedSurf,
            bool useSiftDescriptor,
            double shapeWeight,
            double minShapeVal,
            double minColorVal,
            double minScore,
            bool useOpponentColorSurf,
            bool useCInvariantSurf,
            bool doGeometricRerank,
            double geoRerankInlierThresh);

  // Loads a pre-built index from a given file. This will regenerate
  // all the parameters so that new images will be converted to
  // descriptors consistent with this index.
  void LoadIndexFromFile(const StringPiece& filename);

  // Rebuilds the index
  void RebuildIndex();

  // Takes the current index and saves it to a given file. To be
  // useful, you must have added new labels by sending
  // AddSpeciesIDLabel messages and then calling RebuildIndex()
  void SaveIndexToFile(const StringPiece& filename);

  // Sets up the publishers and subscribers for the ROS node
  void ConnectToROS(ros::NodeHandle& handle);

  // Spins in ROS to handle the messages.
  void Spin() {ros::spin();}

  // Callback that adds a label to the dictionary
  void HandleAddLabel(const reefbot_msgs::AddSpeciesLabel::ConstPtr& msg);

  // Callback for grid service
  bool HandleGridServiceRequest(SpeciesIDGrid::Request& request,
                                SpeciesIDGrid::Response& response);

private:
  ImageSearchIndex index_;

  SpeciesDescriptorParameters params_;

  MultiHistogramDocGenerator histGenerator_;
  RandomDetector randDetector_;
  boost::scoped_ptr<ColorDescriptorGenerator> colorGenerator_;
  boost::scoped_ptr<cv::FeatureDetector> shapeDetector_;
  boost::scoped_ptr<ImageDescriptorGenerator<float> > shapeGenerator_;
  double minScore_; // Minimum score to consider a document

  boost::scoped_ptr<DocSimilarityCalculator> similarityCalculator_;
  boost::scoped_ptr<ReweightDocSimilarity> docReweighter_;

  ros::Publisher publisher_;

  ros::Subscriber requestSub_;

  ros::Subscriber addSpeciesSub_;

  ros::ServiceServer service_;

  ros::ServiceServer gridService_;

  // Callback that handles a SpeciesIDRequest and publishes a
  // SpeciesIDResponse
  void HandleIDRequest(const reefbot_msgs::SpeciesIDRequest::ConstPtr& msg);

  // Callback for the service
  bool HandleIDServiceRequest(SpeciesID::Request& request,
                              SpeciesID::Response& response);

  // Handles the generic version of the id request. Returns true if
  // result is good.
  bool HandleIDRequestImpl(
    const reefbot_msgs::SpeciesIDRequest& request,
    reefbot_msgs::SpeciesIDResponse* response);

  // Converts a set of image regions to documents
  //
  // Inputs:
  // image - The image that needs to be converted
  // regions - The regions in the image to convert
  // 
  // Outputs:
  // documents - List of documents, one for each region
  void ConvertImagesToDocuments(
    const sensor_msgs::Image& image,
    const std::vector<reefbot_msgs::ImageRegion>& regions,
    std::vector<boost::shared_ptr<ImageDocument> >* documents) const;

  void Init(const SpeciesDescriptorParameters& params);
                                
};

} // namespace

BOOST_CLASS_VERSION(species_id::SpeciesDescriptorParameters, 3);

#endif
