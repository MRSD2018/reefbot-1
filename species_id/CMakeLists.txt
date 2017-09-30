cmake_minimum_required(VERSION 2.8)
include(ExternalProject)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(${PROJECT_SOURCE_DIR}/../ROSCommon.cmake)
include(${PROJECT_SOURCE_DIR}/../IncludeMatlab.cmake)

# Allow me to use newer OpenCV modules
#set(OPENCV_INSTALL_DIR ${EXTERNAL_BUILD_DIR}/opencv/install)
#ExternalProject_Add(
#  OpenCV
#  PREFIX ${EXTERNAL_BUILD_DIR}/opencv
#  UPDATE_COMMAND "" # Stop it from updating because we want a static repo
#  SVN_REPOSITORY https://code.ros.org/svn/opencv/trunk/opencv
#  SVN_REVISION "-r3263"
#  CMAKE_ARGS -DCMAKE_BUILD_TYPE:=Debug -DCMAKE_INSTALL_PREFIX:PATH=${OPENCV_INSTALL_DIR}
#  INSTALL_DIR ${OPENCV_INSTALL_DIR}
#)
# -DBUILD_DOXYGEN_DOCS:BOOL=OFF

# Allow me to use flann
set(FLANN_INSTALL_DIR ${EXTERNAL_BUILD_DIR}/flann/install)
ExternalProject_Add(
  flannProj
  PREFIX ${EXTERNAL_BUILD_DIR}/flann
  UPDATE_COMMAND "" # Stop it from updating because we want a static repo
  URL ${PROJECT_SOURCE_DIR}/../../external/flann-1.5
  INSTALL_DIR ${FLANN_INSTALL_DIR}
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${FLANN_INSTALL_DIR}
)

add_subdirectory(../../base ${EXTERNAL_BUILD_DIR}/base)

rosbuild_add_boost_directories()

INCLUDE_DIRECTORIES(
  ${FLANN_INSTALL_DIR}/include
  #${OPENCV_INSTALL_DIR}/include
  ${EXTERNAL_BUILD_DIR}
  ${PROJECT_SOURCE_DIR}/../..
)

LINK_DIRECTORIES(
  #${OPENCV_INSTALL_DIR}/lib
  ${FLANN_INSTALL_DIR}/lib
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#  Profile        : Same as RelWithDebInfo but can use pprof to do profiling
set(ROS_BUILD_TYPE Profile)

rosbuild_init()

# Turn on the fPIC compiler flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()

configure_file(src/ExtractBatchFeatures.sh ${EXECUTABLE_OUTPUT_PATH} COPYONLY)
configure_file(src/MakeDictionary.sh ${EXECUTABLE_OUTPUT_PATH} COPYONLY)
configure_file(src/CreateBatchBlobFrames.sh ${EXECUTABLE_OUTPUT_PATH} COPYONLY)

add_library(ColorConverter src/ColorConverter.cc)
target_link_libraries(ColorConverter opencv_core)

add_library(SIFTDescriptor src/SIFTDescriptor.cc)
target_link_libraries(SIFTDescriptor opencv_core opencv_features2d)
add_dependencies(SIFTDescriptor OpenCV)

rosbuild_add_library(SIFTDescriptorGenerator src/SIFTDescriptorGenerator.cc)
target_link_libraries(SIFTDescriptorGenerator SIFTDescriptor opencv_core)
add_dependencies(SIFTDescriptorGenerator OpenCV)

add_library(SURFDescriptor src/SURFDescriptor.cc)
target_link_libraries(SURFDescriptor opencv_core opencv_features2d)
add_dependencies(SURFDescriptor OpenCV)

rosbuild_add_library(SURFDescriptorGenerator src/SURFDescriptorGenerator.cc)
target_link_libraries(SURFDescriptorGenerator SURFDescriptor opencv_core)
add_dependencies(SURFDescriptorGenerator OpenCV)

rosbuild_add_library(ColorSURFDescriptorGenerator src/ColorSURFDescriptorGenerator.cc)
target_link_libraries(ColorSURFDescriptorGenerator
  SURFDescriptorGenerator
  SURFDescriptor
  ColorConverter
  opencv_core
  opencv_imgproc)
add_dependencies(ColorSURFDescriptorGenerator OpenCV)

add_library(RandomDetector src/RandomDetector.cc)
target_link_libraries(RandomDetector opencv_core opencv_features2d)
add_dependencies(RandomDetector OpenCV)

rosbuild_add_library(ColorDescriptorGenerator src/ColorDescriptorGenerator.cc)
target_link_libraries(ColorDescriptorGenerator
  opencv_core
  opencv_features2d)
add_dependencies(ColorDescriptorGenerator OpenCV)

rosbuild_add_library(HistogramImageDescriptor src/HistogramImageDescriptor.cc)
target_link_libraries(HistogramImageDescriptor
  ImageDocument)

# The HistogramDescriptorGenerator is a template library. If you're
# using it, you need to link to the following libraries:
# - opencv_core, opencv_features2d, HistogramImageDescriptor, flann_cpp
# And add dependencies for
# - OpenCV, flannProj


add_library(ImageDescriptorGeneratorFlags src/ImageDescriptorGeneratorFlags.cc)
target_link_libraries(ImageDescriptorGeneratorFlags
  SIFTDescriptorGenerator
  SURFDescriptorGenerator
  RandomDetector
  ColorDescriptorGenerator
  gflags
  opencv_features2d)

add_library(MultiHistogramDocGenerator src/MultiHistogramDocGenerator.cc)
add_dependencies(MultiHistogramDocGenerator
  OpenCV
  flannProj
)
target_link_libraries(MultiHistogramDocGenerator
  opencv_core
  opencv_features2d
  HistogramImageDescriptor
  flann_cpp
  ImageDocument
  cvutils
)


rosbuild_add_executable(ExtractFeatures src/ExtractFeatures-Main.cc)
target_link_libraries(ExtractFeatures
  opencv_core
  opencv_features2d
  gflags
  blobs
  SIFTDescriptorGenerator
  SURFDescriptorGenerator
  ColorSURFDescriptorGenerator
  ImageDescriptorGeneratorFlags
)

# The search tools
add_library(Term src/Term.cc)
rosbuild_link_boost(Term serialization)

add_library(ImageDocument src/ImageDocument.cc)
rosbuild_link_boost(ImageDocument serialization)
target_link_libraries(ImageDocument Term)

add_library(SearchResult STATIC src/SearchResult.cc)

add_library(ImageSearchIndex 
  src/ImageSearchIndex.cc
  src/DocSimilarityCalculator.cc)
rosbuild_link_boost(ImageSearchIndex serialization)
target_link_libraries(ImageSearchIndex
  ImageDocument
  SearchResult
  gflags
  m
  opencv_core
  opencv_imgproc
  opencv_calib3d)

rosbuild_add_library(SpeciesIDNodeLib src/SpeciesIDNode.cc)
target_link_libraries(SpeciesIDNodeLib
  MultiHistogramDocGenerator
  SIFTDescriptorGenerator
  SURFDescriptorGenerator
  ColorSURFDescriptorGenerator
  ColorDescriptorGenerator
  HistogramImageDescriptor
  RandomDetector
  ImageSearchIndex
  flann_cpp
  opencv_core
  opencv_features2d)
rosbuild_link_boost(SpeciesIDNodeLib serialization)
add_dependencies(SpeciesIDNodeLib OpenCV flannProj)

rosbuild_add_executable(SpeciesIDRosNode src/SpeciesIDNode-Main.cc)
target_link_libraries(SpeciesIDRosNode SpeciesIDNodeLib)

rosbuild_add_executable(BuildFishIndex src/BuildFishIndex.cc)
target_link_libraries(BuildFishIndex
  opencv_core
  opencv_highgui
  gflags
  blobs
  SpeciesIDNodeLib)
add_dependencies(BuildFishIndex OpenCV)

rosbuild_add_library(EvolutionModel src/EvolutionModel.cc)
target_link_libraries(EvolutionModel
  TransformEstimator
  opencv_core
  opencv_imgproc
)

rosbuild_add_library(MotionExtractor src/MotionExtractor.cc)
target_link_libraries(MotionExtractor
  opencv_core
  opencv_imgproc
  blobs
  cvutils
  TransformEstimator
  EvolutionModel
  VisualUtilityFilter)

rosbuild_add_executable(HighlightMotion src/HighlightMotion.cc)
target_link_libraries(HighlightMotion
  MotionExtractor
  EvolutionModel
  TransformEstimator
  VisualUtilityFilter
  gflags
  opencv_highgui
  opencv_imgproc
  cvutils)

#rosbuild_add_executable(ProcessVideoForMotion src/ProcessVideoForMotion.cc)
#target_link_libraries(ProcessVideoForMotion
#  opencv_core
#  opencv_highgui
#  opencv_imgproc
#  MotionExtractor
#  blobs
#  gflags)
  
# All the tests go after this point
include_directories(src)

rosbuild_add_gtest(TermTest test/TermTest.cc)
target_link_libraries(TermTest
  Term
  opencv_core
  opencv_imgproc)

rosbuild_add_gtest(ImageDocumentTest test/ImageDocumentTest.cc)
target_link_libraries(ImageDocumentTest
  ImageDocument
  opencv_core
  opencv_imgproc)

rosbuild_add_gtest(ImageSearchIndexTest test/ImageSearchIndexTest.cc)
target_link_libraries(ImageSearchIndexTest ImageSearchIndex)

rosbuild_add_gtest(HistogramDescriptorGeneratorTest
  test/HistogramDescriptorGeneratorTest.cc)
target_link_libraries(HistogramDescriptorGeneratorTest
  HistogramImageDescriptor
  opencv_core
  opencv_features2d
  flann_cpp)
add_dependencies(HistogramDescriptorGeneratorTest
  OpenCV
  flannProj)

rosbuild_add_gtest(MotionExtractorTest test/MotionExtractorTest.cc)
target_link_libraries(MotionExtractorTest
  MotionExtractor
  opencv_core
  opencv_imgproc)

rosbuild_add_gtest(DocSimilarityTest test/DocSimilarityTest.cc)
target_link_libraries(DocSimilarityTest
  ImageSearchIndex
  opencv_imgproc
  opencv_core)

# All the python builds go after this point
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})
link_libraries(${PYTHON_LIBRARIES})
SET(PYTHON_EXECUTABLE /usr/bin/python)
rosbuild_add_library(cDetectionUtils src/cDetectionUtils.cc)
set_target_properties(cDetectionUtils PROPERTIES OUTPUT_NAME cDetectionUtils PREFIX "" SUFFIX ".so")