cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#find_package(OpenCV 2.3.1 REQUIRED)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/cv_bridge.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS} 
  opencv_core
  opencv_imgproc
  sensor_msgs
  )

# All the tests below here
rosbuild_add_gtest(cv_bridge_test test/cv_bridge_test.cc)
target_link_libraries(cv_bridge_test
  ${PROJECT_NAME}
  )