#include "ImageDescriptorGeneratorFlags.h"

#include <cstdio>
#include <boost/shared_ptr.hpp>
#include "opencv2/imgproc/types_c.h"

#include "ImageDescriptorGenerator.h"
#include "RandomDetector.h"
#include "SIFTDescriptorGenerator.h"
#include "SURFDescriptorGenerator.h"
#include "ColorSURFDescriptorGenerator.h"
#include "ColorDescriptorGenerator.h"
#include "ColorConverter-Inl.h"

using namespace boost;
using namespace cv;
using namespace species_id;

// Of these flags, at exactly one *_descriptor flag should be true and
// if the descriptor uses keypoints, only one *_detector flag should
// be true

// FAST detector from Machine learning for high-speed corner
// detection, E. Rosten and T. Drummond, ECCV 2006
DEFINE_bool(fast_detector, false, "Use the FAST keypoint detector");
DEFINE_int32(fast_threshold, 1, "Threshold for the FAST algorithm");

// Harris detector implemented using the OpenCV GoodFeaturesToTrack
DEFINE_bool(harris_detector, false, "Use the Harris detector");
DEFINE_int32(harris_max_keypoints, 1024,
             "Maximum number of harris points to find in an image");
DEFINE_int32(harris_blocksize, 3, "Block size for the Harris detector");
DEFINE_double(harris_k, 0.04, "Gain for the Harris detector");
DEFINE_double(harris_quality, 0.8, "Harris detector threshold as "
              "percentage of the maximum response");
DEFINE_double(harris_min_distance, 32, "How far apart the keypoints must be");

// Star Detector by K. Konolige. It is a multiscale center-surround detector.
DEFINE_bool(star_detector, false, "Use the Star detector by K. Konolige");
DEFINE_int32(star_max_size, 16, "Star detector max size");
DEFINE_int32(star_response_threshold, 30, "Star detector response threshold");
DEFINE_int32(star_line_threshold, 10, "Star detector line threshold");
DEFINE_int32(star_line_threshold_binary, 10, "Star detector line threshold "
             "in the closest power of 2");
DEFINE_int32(star_nonmax_size, 5, "Star detector threshold for suppresssing"
             " the non-maximal value");

// SIFT Detector/Descriptor
DEFINE_bool(sift_detector, false, "Use the SIFT detector to find keypoints");
DEFINE_bool(sift_descriptor, false,
            "Use the SIFT descriptor to describe keypoints");
DEFINE_double(sift_threshold, cv::SIFT::DetectorParams::
              GET_DEFAULT_THRESHOLD(),
              "Threshold for the SIFT detector");
DEFINE_double(sift_edge_threshold, cv::SIFT::DetectorParams::
              GET_DEFAULT_EDGE_THRESHOLD(),
              "Edge threshold for the SIFT detector");
DEFINE_int32(sift_octaves, cv::SIFT::CommonParams::DEFAULT_NOCTAVES,
             "Number of SIFT octaves to use");
DEFINE_int32(sift_octave_layers, cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, 
             "Number of SIFT octave layers to use");
DEFINE_int32(sift_first_octave, cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
             "First SIFT octave to use");
DEFINE_int32(sift_angle_mode, cv::SIFT::CommonParams::FIRST_ANGLE,
             "SIFT angle mode");
DEFINE_double(sift_magnification,
              cv::SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(),
              "SIFT magnification parameter");
DEFINE_bool(sift_normalized, true, "Are the SIFT values normalized?");

// SURF Feature Detector (uses the hessian mostly) and Descriptor
DEFINE_bool(surf_detector, false,
            "Use the SURF (Hessian) detector to find keypoints");
DEFINE_bool(surf_descriptor, false,
            "Use the SURF descriptor to describe keypoints");
DEFINE_double(surf_hessian_threshold, 400, "Surf Hessian Threshold");
DEFINE_int32(surf_octaves, 3, "Number of SURF octaves to use");
DEFINE_int32(surf_octave_layers, 4, "Number of SURF octave layers to use");
DEFINE_bool(surf_extended, false,
            "Use the extended (128 entry) version of SURF?");

// Color SURF Descriptors
DEFINE_bool(opponent_color_surf, false, "Do we use the opponent color SURF descriptor?");
DEFINE_bool(cinvariant_color_surf, false, "Do we use the C-SURF descriptor?");

// A random keypoint selector. This is useful if you want a random
// sampling of the descriptors in the image.
DEFINE_bool(random_detector, false,
            "Use a randomized detector to find keypoints");
DEFINE_double(random_frac, 0.01,
              "Fraction of pixels in the image to choose randomly");
DEFINE_int64(random_seed, 123456, "Seed for the random number generator");

// HSV color descriptor which is just the color at that pixel
DEFINE_bool(color_descriptor, false,
            "Use the color of the pixel as the descriptor");
DEFINE_string(color_converter, "NO_COLOR_CVT",
              "The type of color conversion to perform. See cvtColor");

void VerifyDescriptorFlags() {
  int numDescriptorsSelected = 
    FLAGS_color_descriptor +
    FLAGS_surf_descriptor +
    FLAGS_sift_descriptor +
    FLAGS_opponent_color_surf +
    FLAGS_cinvariant_color_surf;
  if (numDescriptorsSelected != 1) {
    std::cerr << "Must select one and only one descriptor" << std::endl;
    exit(2);
  }

  int numDetectorsSelected = 
    FLAGS_sift_detector +
    FLAGS_surf_detector +
    FLAGS_random_detector +
    FLAGS_star_detector +
    FLAGS_harris_detector +
    FLAGS_fast_detector;
  if (numDetectorsSelected != 1) {
    std::cerr << "Must select one and only one detector" << std::endl;
    exit(2);
  }
}

FeatureDetector* ChooseFeatureDetector() {
  if (FLAGS_fast_detector) {
    return new FastFeatureDetector(FLAGS_fast_threshold, true);
  } else if (FLAGS_harris_detector) {
    return new GoodFeaturesToTrackDetector(FLAGS_harris_max_keypoints,
                                           FLAGS_harris_quality,
                                           FLAGS_harris_min_distance,
                                           FLAGS_harris_blocksize,
                                           true, // use harris
                                           FLAGS_harris_k);
  } else if (FLAGS_star_detector) {
    return new StarFeatureDetector(FLAGS_star_max_size,
                                   FLAGS_star_response_threshold,
                                   FLAGS_star_line_threshold,
                                   FLAGS_star_line_threshold_binary,
                                   FLAGS_star_nonmax_size);
  } else if (FLAGS_sift_detector) {
    return new SiftFeatureDetector(FLAGS_sift_threshold,
                                   FLAGS_sift_edge_threshold,
                                   FLAGS_sift_octaves,
                                   FLAGS_sift_octave_layers,
                                   FLAGS_sift_first_octave,
                                   FLAGS_sift_angle_mode);
  } else if (FLAGS_surf_detector) {
    return new SurfFeatureDetector(FLAGS_surf_hessian_threshold,
                                   FLAGS_surf_octaves,
                                   FLAGS_surf_octave_layers);
  } else if (FLAGS_random_detector) {
    return new RandomDetector(FLAGS_random_frac,
                              FLAGS_random_seed);
  }
  return NULL;
}

ImageDescriptorGenerator<float>* ChooseImageDescriptor(
    const FeatureDetector* detector) {
  if (FLAGS_sift_descriptor) {
    return new SIFTDescriptorGenerator(detector,
                                       FLAGS_sift_magnification,
                                       FLAGS_sift_normalized,
                                       true, // recalculated angles
                                       FLAGS_sift_octaves,
                                       FLAGS_sift_octave_layers,
                                       FLAGS_sift_first_octave,
                                       FLAGS_sift_angle_mode);
  } else if (FLAGS_surf_descriptor) {
    return new SURFDescriptorGenerator(detector,
                                       FLAGS_surf_octaves,
                                       FLAGS_surf_octave_layers,
                                       FLAGS_surf_extended);
  } else if (FLAGS_color_descriptor) {
    int colorConversion =
      ColorDescriptorGenerator::ConversionCodeStringToInt(
        FLAGS_color_converter);

    return new ColorDescriptorGenerator(detector, colorConversion);
  } else if (FLAGS_opponent_color_surf) {
    return new ColorSURFDescriptorGenerator(
      shared_ptr<ColorConverter>(new OpponentColorConverter<uint8>()),
      detector,
      FLAGS_surf_octaves,
      FLAGS_surf_octave_layers,
      FLAGS_surf_extended);
  } else if (FLAGS_cinvariant_color_surf) {
    return new ColorSURFDescriptorGenerator(
      shared_ptr<ColorConverter>(new CInvariantColorConverter<uint8>()),
      detector,
      FLAGS_surf_octaves,
      FLAGS_surf_octave_layers,
      FLAGS_surf_extended);
  }
  return NULL;
}
