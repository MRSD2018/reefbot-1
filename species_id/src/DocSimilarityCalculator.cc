#include "DocSimilarityCalculator.h"

#include <ros/ros.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

namespace species_id {

DocSimilarityCalculator::~DocSimilarityCalculator(){}

CosineSimilarity::~CosineSimilarity(){}


double CosineSimilarity::CalcDocumentSimilarity(const ImageDocument& a,
                                                const ImageDocument& b) const{
  double score = 0;

  // Start by calculating the dot product by iterating through the smaller doc
  const ImageDocument* iterDoc = &a;
  const ImageDocument* lookupDoc = &b;
  if (a.nUniqueTerms() > b.nUniqueTerms()) {
    iterDoc = &b;
    lookupDoc = &a;
  }
  for(ImageDocument::TermMap::const_iterator termI = iterDoc->begin();
      termI != iterDoc->end(); ++termI) {
    score += (termI->second->val() * lookupDoc->GetVal(termI->first));
  }

  // Now divide by the lengths
  score /= a.length();
  score /= b.length();

  return score;
}

ReweightDocSimilarity::~ReweightDocSimilarity() {}

PerspectiveGeometricReweight::~PerspectiveGeometricReweight() {}

double PerspectiveGeometricReweight::CalcNewSimilarity(
  const ImageDocument& a,
  const ImageDocument& b,
  double oldSimilarity) const {

  // First load all the matching pairs of points into vectors
  vector<Point2f> aPoints;
  vector<Point2f> bPoints;
  const ImageDocument* iterDoc = &a;
  const ImageDocument* lookupDoc = &b;
  if (a.nUniqueTerms() > b.nUniqueTerms()) {
    iterDoc = &b;
    lookupDoc = &a;
  }
  for(ImageDocument::TermMap::const_iterator termI = iterDoc->begin();
      termI != iterDoc->end(); ++termI) {
    const Term* bTerm = lookupDoc->GetTerm(termI->first);
    if (bTerm == NULL) {
      continue;
    }
    const Term* aTerm = termI->second.get();

    ROS_ASSERT(aTerm->coords() != NULL &&
               bTerm->coords() != NULL);

    const std::deque<Term::Coord>& aCoords = *aTerm->coords();
    const std::deque<Term::Coord>& bCoords = *bTerm->coords();
    for (std::deque<Term::Coord>::const_iterator aCoord = aCoords.begin();
         aCoord != aCoords.end(); aCoord++) {
      for (std::deque<Term::Coord>::const_iterator bCoord = bCoords.begin();
         bCoord != bCoords.end(); bCoord++) {
        aPoints.push_back(Point2f(aCoord->first, aCoord->second));
        bPoints.push_back(Point2f(bCoord->first, bCoord->second));
      }
    }
  }

  // If there aren't enough pairs of points to calculate a homography
  // transform, we're done.
  if (aPoints.size() < 6) {
    return oldSimilarity;
  }

  ROS_INFO_STREAM("Found enough points to try reranking " << aPoints.size());

  // Now copy the points to a matrix
  Mat_<double> aPointsMat(3, aPoints.size(), 1.0);
  Mat_<Vec2f> aPointsChanMat(aPoints.size(), 1);
  Mat_<double> bPointsMat(3, bPoints.size(), 1.0);
  Mat_<Vec2f> bPointsChanMat(aPoints.size(), 1);
  for (int i = 0; i < aPoints.size(); i++) {
    aPointsMat[0][i] = aPoints[i].x;
    aPointsMat[1][i] = aPoints[i].y;
    aPointsChanMat[i][0][0] = aPoints[i].x;
    aPointsChanMat[i][0][1] = aPoints[i].y;

    bPointsMat[0][i] = bPoints[i].x;
    bPointsMat[1][i] = bPoints[i].y;
    bPointsChanMat[i][0][0] = bPoints[i].x;
    bPointsChanMat[i][0][1] = bPoints[i].y;

  }

  // Now, calculate the best homography for points in order
  // to find the geometric consistency. The best one is the one with
  // the maximum number of inliers.
  Mat H = findHomography(aPointsChanMat, bPointsChanMat, CV_RANSAC, inlierThresh_);

  // Count the number of inliers
  Mat projectedPoints = H * aPointsMat;
  projectedPoints.row(0) = projectedPoints.row(0) / 
    projectedPoints.row(2);
  projectedPoints.row(1) = projectedPoints.row(1) / 
    projectedPoints.row(2);
  
  Mat distances = projectedPoints - bPointsMat;
  distances = distances.mul(distances);
  distances = distances.row(0) + distances.row(1);
  int inliers = countNonZero(distances < inlierThresh_);
  
  if (inliers > 0) {
    ROS_INFO_STREAM("Found " << inliers << " inliers.");
  }

  if (inliers - 5 > 0) {
    ROS_INFO_STREAM("Found " << inliers-5 << " inliers. Reranking to "
                    << inliers - 5 + oldSimilarity);
    return inliers - 5 + oldSimilarity;
  }
  
  return oldSimilarity;
  
}

} // namespace
