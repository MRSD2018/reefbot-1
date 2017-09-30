// ROS Node that loads up a search index and then acts as a ROS Node
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

#include "SpeciesIDNode.h"
#include <algorithm>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/serialization/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>
#include "base/hash_map.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/types_c.h"

#include "reefbot_msgs/SpeciesIDResponse.h"
#include "reefbot_msgs/SpeciesScore.h"

#include "HistogramDescriptorGenerator-Inl.h"
#include "SIFTDescriptorGenerator.h"
#include "SURFDescriptorGenerator.h"
#include "ColorSURFDescriptorGenerator.h"
#include "ColorConverter-Inl.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"


using namespace std;
using namespace boost;
using namespace ros;
using namespace reefbot_msgs;
using cv_bridge::CvMultiMatConstPtr;
namespace enc = sensor_msgs::image_encodings;
using cv::Mat;
using cv::Mat_;
using cv::Rect;

namespace species_id {

ostream& operator<< (ostream& out, const SpeciesDescriptorParameters& s) {
  return out
    << "version: " << s.version() << endl
    << "colorDictFilename: " << s.colorDictFilename << endl
    << "colorConvertStr: " << s.colorConverterStr << endl
    << "colorFrac: " << s.colorFrac << endl
    << "shapeDictFilename: " << s.shapeDictFilename << endl
    << "useSurfDetector: " << s.useSurfDetector << endl
    << "hessianThreshold: " << s.hessianThreshold << endl
    << "octaves: " << s.octaves << endl
    << "octaveLayers: " << s.octaveLayers << endl
    << "useSurfDescriptor: " << s.useSurfDescriptor << endl
    << "extendedSurf: " << s.extendedSurf << endl
    << "useSiftDescriptor: " << s.useSiftDescriptor << endl
    << "shapeWeight: " << s.shapeWeight << endl
    << "minShapeVal: " << s.minShapeVal << endl
    << "minColorVal: " << s.minColorVal << endl
    << "minScore: " << s.minScore << endl
    << "useOpponentColorSurf: " << s.useOpponentColorSurf << endl
    << "useCInvariantSurf: " << s.useCInvariantSurf << endl
    << "doGeometricRerank: " << s.doGeometricRerank << endl
    << "geoRerankInlierThresh: " << s.geoRerankInlierThresh << endl;
}

bool IsParamTrue(const NodeHandle& node, const char* paramName) {
  bool retVal = false;
  node.param(paramName, retVal, false);
  return retVal;
}

SpeciesIDNode::SpeciesIDNode()
  : index_(),
    params_(),
    histGenerator_(),
    randDetector_(),
    colorGenerator_(NULL),
    shapeDetector_(),
    shapeGenerator_() {}

void SpeciesIDNode::Init(const StringPiece& colorDictFilename,
                         const StringPiece& colorConverterStr,
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
                         double geoRerankInlierThresh) {

  params_ = SpeciesDescriptorParameters(colorDictFilename,
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

  Init(params_);

}

void SpeciesIDNode::Init(const SpeciesDescriptorParameters& params) {
  ROS_ASSERT(params.version() > 0);

  ROS_INFO_STREAM("Intializing ROS node with params:" << endl << params);

  // Choose the color descriptors if we're using them
  if (params.colorDictFilename != "" &&
      params.colorConverterStr != "" &&
      params.colorFrac > 0) {

    // OpenCV color converter string
    int colorConverter;
    colorConverter = ColorDescriptorGenerator::ConversionCodeStringToInt(
          params.colorConverterStr);

    randDetector_ = RandomDetector(params.colorFrac);
    colorGenerator_.reset(new ColorDescriptorGenerator(&randDetector_,
                                                       colorConverter));

    // Add the color histogram generator
    // The flann parameters were found by autotuning once and then
    // copying the resulting parameters.
    histGenerator_.AddHistogramType(
      HistogramDescriptorGenerator<float>::CreateFromINRIADictionary(
        colorGenerator_.get(),
        params.colorDictFilename,
        flann::LinearIndexParams(),
        32, // checks
        params.doGeometricRerank), // trackCoords
      params.minColorVal,
      1-params.shapeWeight);
  }

  // Choose the keypoint detector
  if (params.useSurfDetector) {
    shapeDetector_.reset(new cv::SurfFeatureDetector(params.hessianThreshold,
                                                     params.octaves,
                                                     params.octaveLayers));
  } else {
    ROS_FATAL("You need to pick a valid keypoint detector");
    exit(3);
  }

  // Choose the descriptor generator
  if (params.useSurfDescriptor) {
    shapeGenerator_.reset(new SURFDescriptorGenerator(shapeDetector_.get(),
                                                      params.octaves,
                                                      params.octaveLayers,
                                                      params.extendedSurf));
  } else if (params.useSiftDescriptor) {
    shapeGenerator_.reset(new SIFTDescriptorGenerator(shapeDetector_.get()));
  } else if (params.useOpponentColorSurf) {
    shapeGenerator_.reset(new ColorSURFDescriptorGenerator(
      shared_ptr<ColorConverter>(new OpponentColorConverter<uint8>()),
      shapeDetector_.get(),
      params.octaves,
      params.octaveLayers,
      params.extendedSurf));
  } else if (params.useCInvariantSurf) {
    shapeGenerator_.reset(new ColorSURFDescriptorGenerator(
      shared_ptr<ColorConverter>(new CInvariantColorConverter<uint8>()),
      shapeDetector_.get(),
      params.octaves,
      params.octaveLayers,
      params.extendedSurf));
  } else {
    ROS_FATAL("You need to pick a valid shape descriptor generator");
    exit(3);
  }

  // Add the shape histogram generator.
  // The flann parameters were found by autotuning once and then
  // copying the resulting parameters.
  histGenerator_.AddHistogramType(
    HistogramDescriptorGenerator<float>::CreateFromINRIADictionary(
      shapeGenerator_.get(),
      params.shapeDictFilename,
      flann::KMeansIndexParams(16, 15, CENTERS_KMEANSPP, 0.4),
      180, // checks
      params.doGeometricRerank), // trackCoords
    params.minShapeVal,
    params.shapeWeight);

  minScore_ = params.minScore;

  // Pick the scoring algorithm
  similarityCalculator_.reset(new CosineSimilarity());
  if (params.doGeometricRerank) {
    docReweighter_.reset(
      new PerspectiveGeometricReweight(params.geoRerankInlierThresh));
  }
}

void SpeciesIDNode::LoadIndexFromFile(const StringPiece& filename) {

  ROS_INFO_STREAM("Loading species index from: " << filename.as_string());

  fstream stream(filename.as_string().c_str(), ios_base::in);
  archive::xml_iarchive archive(stream);
  archive >> boost::serialization::make_nvp("index_", index_);
  archive >> boost::serialization::make_nvp("params_", params_);

  ROS_INFO_STREAM("Finished loading index");

  Init(params_);

  stream.close();
}

void SpeciesIDNode::RebuildIndex() {
  ROS_INFO("Rebuilding the image search index");
  index_.BuildIndex();
}

void SpeciesIDNode::SaveIndexToFile(const StringPiece& filename) {

  ROS_INFO_STREAM("Saving the current index to " << filename.as_string());

  fstream stream(filename.as_string().c_str(),
                 ios_base::out | ios_base::trunc);
  archive::xml_oarchive archive(stream);
  archive << BOOST_SERIALIZATION_NVP(index_);
  archive << BOOST_SERIALIZATION_NVP(params_);

  ROS_INFO_STREAM("Finished saving index");
}

void SpeciesIDNode::ConnectToROS(ros::NodeHandle& handle) {
  string requestTopic;
  string responseTopic;
  string addLabelTopic;
  string serviceName;

  // Setup a response to direct request messages
  handle.param<string>("species_id_request_topic", requestTopic,
                       "request_species_id");
  requestSub_ = handle.subscribe<reefbot_msgs::SpeciesIDRequest, SpeciesIDNode>(
    requestTopic, 8,
    &SpeciesIDNode::HandleIDRequest,
    this);

  handle.param<string>("species_id_response_topic", responseTopic,
                       "species_id");
  publisher_ = handle.advertise<reefbot_msgs::SpeciesIDResponse>(
    responseTopic, 8, true);

  handle.param<string>("add_label_topic", addLabelTopic,
                       "add_species_label");
  addSpeciesSub_ = handle.subscribe(addLabelTopic, 128,
                                    &SpeciesIDNode::HandleAddLabel,
                                    this);

  // Setup a ROS Service that handles requests
  ros::NodeHandle privateHandle("~");
  privateHandle.param<string>("species_id_service", serviceName,
                              "species_id_service");
  service_ = handle.advertiseService(
    serviceName,
    &SpeciesIDNode::HandleIDServiceRequest,
    this);

  // Setup the service for grid requests
  gridService_ = handle.advertiseService(
    "species_id_grid_service",
    &SpeciesIDNode::HandleGridServiceRequest, this);
}

// Comparator to sort scores in descending order
bool SpeciesScoreComparator(SpeciesScore a, SpeciesScore b) {
  return a.score > b.score;
}

void SpeciesIDNode::HandleIDRequest(const reefbot_msgs::SpeciesIDRequest::ConstPtr& msg) {

  reefbot_msgs::SpeciesIDResponse response;
  if (HandleIDRequestImpl(*msg, &response)) {
    publisher_.publish(response);
  }
}

bool SpeciesIDNode::HandleIDServiceRequest(SpeciesID::Request& request,
                                           SpeciesID::Response& response) {
  return HandleIDRequestImpl(request.request, &response.response);
}

bool SpeciesIDNode::HandleIDRequestImpl(
                                        const reefbot_msgs::SpeciesIDRequest& request,
                                        reefbot_msgs::SpeciesIDResponse* response) {
  vector<shared_ptr<ImageDocument> > documents;

  ConvertImagesToDocuments(request.image, request.regions, &documents);

  if (documents.size() != request.regions.size()) {
    ROS_ERROR("Number of results did not match the regions");
    return false;
  }

  // Put together the pieces for the response
  response->image_id = request.image_id;
  vector<ImageRegion>::const_iterator regionI = request.regions.begin();
  for (vector<shared_ptr<ImageDocument> >::const_iterator docI =
         documents.begin();
       docI != documents.end();
       ++docI, ++regionI) {
    SingleSpeciesId answer;
    answer.bounding_box = regionI->bounding_box;
    
    // Search for matching documents
    vector<SearchResult> results;
    index_.Search(**docI, &results, *similarityCalculator_,
                  docReweighter_.get());

    // Now load the resulting hits and and sort them
    for (vector<SearchResult>::iterator resultI = results.end();
         resultI != results.begin();) {
      pop_heap(results.begin(), resultI);
      --resultI;

      if (resultI->score() < minScore_) {
        // Score are too low so we ignore the other results
        break;
      }
      SpeciesScore result;
      result.score = resultI->score();
      result.species_id = lexical_cast<uint32>(resultI->doc()->label());
      result.meta_data = resultI->doc()->metaData();
      answer.best_species.push_back(result);
    }
    //TODO(mdesnoyer): I don't think I need to sort here because we
    //were popping off a heap and thus the sorting happens
    //automagically.
    //sort(answer.best_species.begin(), answer.best_species.end(),
    //     SpeciesScoreComparator);
    
    response->answers.push_back(answer);
  }

  return true;
}

void GetGridHeightsAndWidths(int imageWidth, int imageHeight,
                             int minX, int minY,
                             int minW, int minH,
                             double strideW,
                             double strideH,
                             bool fixAspect,
                             vector<double>* widths,
                             vector<double>* heights) {
  ROS_ASSERT(widths);
  ROS_ASSERT(heights);

  if (fixAspect) {
    strideH = strideW;
  }

  for (double curW = minW; curW < imageWidth-minX; curW *= strideW) {
    widths->push_back(cvRound(curW));
  }

  for (double curH = minH; curH < imageHeight-minY; curH *= strideH) {
    heights->push_back(cvRound(curH));
  }

  // If the aspect ratio is fixed, truncate the list of sizes to the
  // smaller one.
  if (fixAspect) {
    if (heights->size() > widths->size()) {
      heights->resize(widths->size());
    } else {
      widths->resize(heights->size());
    }
  }
}

template <typename T>
Mat InitializeGridResponse(int imageWidth, int imageHeight,
                           int minX, int minY,
                           int strideX, int strideY,
                           bool fixAspect,
                           const vector<double>& widths,
                           const vector<double>& heights,
                           int cvType,
                           const T& defaultVal) {
  vector<int> scoreSize;
  scoreSize.push_back(widths.size());
  if (!fixAspect) scoreSize.push_back(heights.size());
  scoreSize.push_back(
    std::max<int>(1, (imageWidth - minX - widths[0]) / strideX));
  scoreSize.push_back(
    std::max<int>(1, (imageHeight - minY - heights[0]) / strideY));
  return Mat(scoreSize.size(), &scoreSize[0], cvType,
             defaultVal);
}

void GetLocationsForEvaluation(int imageWidth, int imageHeight,
                               int minX, int minY,
                               int strideX, int strideY,
                               double width, double height,
                               const Mat& mask,
                               const Mat& confidenceSub,
                               const float* const confidenceStart,
                               vector<shared_ptr<ImageRegion> >* regions,
                               vector<int64_t>* regionOffset) {
  ROS_ASSERT(mask.empty() || mask.type() == CV_8U);
  ROS_ASSERT(confidenceSub.type() == CV_32F);
  ROS_ASSERT(regions && regionOffset);

  for (int i = 0, curX = minX; curX < imageWidth-width; ++i) {
    for (int j = 0, curY = minY; curY < imageHeight-height; ++j) {
      if (mask.empty() || mask.at<uint8_t>(i,j)) {
        regions->push_back(shared_ptr<ImageRegion>(new ImageRegion()));
        sensor_msgs::RegionOfInterest& roi = regions->back()->bounding_box;
        roi.x_offset = curX;
        roi.y_offset = curY;
        roi.height = height;
        roi.width = width;
        regionOffset->push_back(confidenceSub.ptr<float>(i,j) -
                                confidenceStart);
      }
      curY += strideY;
    }
    curX += strideX;
  }
}

void InitializeGridOutputsAndListRegions(const sensor_msgs::Image& image,
                                         const objdetect_msgs::Grid& grid,
                                         const sensor_msgs::MatND& maskMsg,
                                         vector<shared_ptr<ImageRegion> >* regions,
                                         vector<int64_t>* regionOffset,
                                         Mat* confidence,
                                         Mat* topLabels,
                                         Mat* notFishConfidence) {
  ROS_ASSERT(regions && regionOffset && confidence && topLabels &&
             notFishConfidence);

  boost::shared_ptr<void const> objTracker;
  CvMultiMatConstPtr maskWrapper = cv_bridge::toCvShare(maskMsg,
                                                        objTracker,
                                                        enc::TYPE_8U);
  const Mat& mask(maskWrapper->mat);
  

  // First figure out the heights and widths defined by the grid
  vector<double> widths;
  vector<double> heights;
  GetGridHeightsAndWidths(image.width,
                          image.height,
                          grid.minX, grid.minY,
                          grid.minW, grid.minH,
                          grid.strideW, grid.strideH,
                          grid.fixAspect, &widths, &heights);

  // Initialize the output matrices
  *confidence = InitializeGridResponse(image.width, image.height,
                                       grid.minX, grid.minY,
                                       grid.strideX, grid.strideY,
                                       grid.fixAspect,
                                       widths, heights,
                                       CV_32F,
                                       numeric_limits<float>::signaling_NaN());
  *topLabels = InitializeGridResponse(image.width, image.height,
                                      grid.minX, grid.minY,
                                      grid.strideX, grid.strideY,
                                      grid.fixAspect,
                                      widths, heights,
                                      CV_8S, -2);
  *notFishConfidence = InitializeGridResponse(image.width, image.height,
                                              grid.minX, grid.minY,
                                              grid.strideX, grid.strideY,
                                              grid.fixAspect,
                                              widths, heights,
                                              CV_32F,
                                              numeric_limits<float>::signaling_NaN());

  ROS_ASSERT(confidence->isContinuous());
  
  // Build the list of regions to evaluate
  for (unsigned int widthI = 0u; widthI < widths.size(); ++widthI) {
    if (grid.fixAspect) {
      GetLocationsForEvaluation(image.width, image.height,
                                grid.minX, grid.minY,
                                grid.strideX, grid.strideY,
                                widths[widthI], heights[widthI],
                                (mask.empty() ? mask :
                                 Mat(2, &mask.size[1], mask.type(),
                                     const_cast<uchar*>(mask.ptr(widthI)),
                                     &mask.step[1])),
                                Mat(2, &confidence->size[1], CV_32F,
                                    confidence->ptr(widthI),
                                    &confidence->step[1]),
                                confidence->ptr<float>(0),
                                regions,
                                regionOffset);
    } else {
      for (unsigned int heightI = 0u; heightI < heights.size(); ++heightI) {
        GetLocationsForEvaluation(image.width, image.height,
                                  grid.minX, grid.minY,
                                  grid.strideX, grid.strideY,
                                  widths[widthI], heights[heightI],
                                  (mask.empty() ? mask :
                                   Mat(2, &mask.size[2], mask.type(),
                                       const_cast<uchar*>(mask.ptr(widthI,
                                                                   heightI)),
                                       &mask.step[2])),
                                  Mat(2, &confidence->size[2], CV_32F,
                                      confidence->ptr(widthI, heightI),
                                      &confidence->step[2]),
                                  confidence->ptr<float>(0),
                                  regions,
                                  regionOffset);
      }
    }
  }
}

bool SpeciesIDNode::HandleGridServiceRequest(
  SpeciesIDGrid::Request& request,
  SpeciesIDGrid::Response& response) {

  // Start by generating the set of regions encapsulated in the grid message
  vector<shared_ptr<ImageRegion> > regions;
  vector<int64_t> regionOffset; // Offset in entries on the flattend grid for each region.
  Mat confidence;
  Mat topLabels;
  Mat notFishConfidence;
  InitializeGridOutputsAndListRegions(request.request.image,
                                      request.request.grid,
                                      request.request.mask,
                                      &regions,
                                      &regionOffset,
                                      &confidence,
                                      &topLabels,
                                      &notFishConfidence);

  ROS_ASSERT(regions.size() == regionOffset.size());

  // Convert the image to OpenCV format
  boost::shared_ptr<void const> objTracker;
  cv_bridge::CvImageConstPtr cvImageWrap = 
    cv_bridge::toCvShare(request.request.image, objTracker, "bgr8");

  // Build a mask for where in the image descriptors will be evaluated
  Mat_<uint8> regionMask = Mat_<uint8>::zeros(cvImageWrap->image.rows,
                                              cvImageWrap->image.cols);
  for(vector<shared_ptr<ImageRegion> >::const_iterator regionI = regions.begin();
      regionI != regions.end(); ++regionI) {
    const sensor_msgs::RegionOfInterest& bbox = (*regionI)->bounding_box;
    regionMask(Rect(bbox.x_offset, bbox.y_offset, bbox.width, bbox.height)) = 1;
  }

  // Start the timer
  ros::Time startTime = ros::Time::now();

  // Initialize the document generator with the current image
  histGenerator_.InitializeForWholeImage(cvImageWrap->image, regionMask);

  // Search for each of the regions record the results
  vector<int64_t>::const_iterator offsetI = regionOffset.begin();
  vector<SearchResult> results;
  for(vector<shared_ptr<ImageRegion> >::const_iterator regionI = regions.begin();
      regionI != regions.end();
      ++regionI, ++offsetI) {
    results.clear();

    // Convert the region into a document
    shared_ptr<ImageDocument> doc = 
      histGenerator_.CreateDocFromSingleMask(cvImageWrap->image,
                                             **regionI);
    
    // Search for the matching documents
    index_.Search(*doc, &results, *similarityCalculator_,
                  docReweighter_.get());

    // Now go through the results and find the first answer that gives
    // it a fish species classification as well as the first result
    // that says it is not a fish.
    bool foundValidFish = false;
    bool foundNonFish = false;
    for (vector<SearchResult>::iterator resultI = results.end();
         resultI != results.begin() && !(foundNonFish && foundValidFish);) {
      pop_heap(results.begin(), resultI);
      --resultI;

      uint32 curLabel = lexical_cast<uint32>(resultI->doc()->label());

      if (!foundValidFish) {
        if (curLabel >= 4) {
          foundValidFish = true;
          *(topLabels.ptr<int8>(0) + *offsetI) = curLabel;
          *(confidence.ptr<float>(0) + *offsetI) = resultI->score();
        }
      }

      if (!foundNonFish) {
        if (curLabel == 1) {
          foundNonFish = true;
          *(notFishConfidence.ptr<float>(0) + *offsetI) = resultI->score();
        }
      }

      if (resultI->score() < minScore_) {
        // Score are too low so we ignore the other results
        break;
      }
    }
  }

  // Turn off the timer
  ros::Duration processingTime = ros::Time::now() - startTime;

  // Build up the output response
  response.processing_time.data = processingTime;
  response.grid.minX = request.request.grid.minX;
  response.grid.minY = request.request.grid.minY;
  response.grid.strideX = request.request.grid.strideX;
  response.grid.strideY = request.request.grid.strideY;
  response.grid.minW = request.request.grid.minW;
  response.grid.minH = request.request.grid.minH;
  response.grid.strideW = request.request.grid.strideW;
  response.grid.strideH = request.request.grid.strideH;
  response.grid.fixAspect = request.request.grid.fixAspect;
  
  cv_bridge::CvMultiMat multiMatConverter;
  multiMatConverter.header.stamp = request.request.header.stamp;
  multiMatConverter.header.frame_id = request.request.header.frame_id;
  multiMatConverter.header.seq = request.request.header.seq;
  
  multiMatConverter.mat = confidence;
  multiMatConverter.toMsg(response.confidence);
  
  multiMatConverter.mat = topLabels;
  multiMatConverter.toMsg(response.top_label);
  
  multiMatConverter.mat = notFishConfidence;
  multiMatConverter.toMsg(response.not_fish_confidence);

  return true;
}

void SpeciesIDNode::HandleAddLabel(const AddSpeciesLabel::ConstPtr& msg) {
  if (msg->regions.size() != msg->labels.size()) {
    ROS_ERROR("The number of labels and regions don't match");
    return;
  }

  ROS_INFO_STREAM("Adding document " << msg->filename);

  vector<shared_ptr<ImageDocument> > documents;

  ConvertImagesToDocuments(msg->image, msg->regions, &documents);

  if (documents.size() != msg->regions.size()) {
    ROS_ERROR("Number of results did not match the regions");
    return;
  }

  // Add the documents to the index
  vector<shared_ptr<ImageDocument> >::iterator docI = documents.begin();
  for (uint32 i = 0; i < documents.size(); ++docI, i++) {
    (*docI)->SetLabel(lexical_cast<string>(msg->labels[i]));

    if (msg->filename.size() > 0 && msg->blob_ids.size() > i) {
      string blobData = msg->filename + '.' + lexical_cast<string>(msg->blob_ids[i]);
      (*docI)->SetMetaData(blobData);
    }

    index_.AddDocument(*docI);
  }
}

void SpeciesIDNode::ConvertImagesToDocuments(
    const sensor_msgs::Image& image,
    const vector<ImageRegion>& regions,
    vector<shared_ptr<ImageDocument> >* documents) const {
  ROS_ASSERT(documents != NULL);

  // Convert the image to OpenCV format
  boost::shared_ptr<void const> objTracker;
  cv_bridge::CvImageConstPtr cvImageWrap = 
    cv_bridge::toCvShare(image, objTracker, "bgr8");

  // Now generate the documents
  histGenerator_.CreateDocsUsingMasks(cvImageWrap->image, regions, documents);
}


} // namespace

