FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sensor_msgs/msg"
  "../src/sensor_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/sensor_msgs/CompressedImage.h"
  "../msg_gen/cpp/include/sensor_msgs/NavSatStatus.h"
  "../msg_gen/cpp/include/sensor_msgs/MatND.h"
  "../msg_gen/cpp/include/sensor_msgs/CameraInfo.h"
  "../msg_gen/cpp/include/sensor_msgs/LaserScan.h"
  "../msg_gen/cpp/include/sensor_msgs/Imu.h"
  "../msg_gen/cpp/include/sensor_msgs/PointCloud2.h"
  "../msg_gen/cpp/include/sensor_msgs/JoyFeedback.h"
  "../msg_gen/cpp/include/sensor_msgs/JoyFeedbackArray.h"
  "../msg_gen/cpp/include/sensor_msgs/PointField.h"
  "../msg_gen/cpp/include/sensor_msgs/RegionOfInterest.h"
  "../msg_gen/cpp/include/sensor_msgs/JointState.h"
  "../msg_gen/cpp/include/sensor_msgs/NavSatFix.h"
  "../msg_gen/cpp/include/sensor_msgs/PointCloud.h"
  "../msg_gen/cpp/include/sensor_msgs/Joy.h"
  "../msg_gen/cpp/include/sensor_msgs/Range.h"
  "../msg_gen/cpp/include/sensor_msgs/Image.h"
  "../msg_gen/cpp/include/sensor_msgs/ChannelFloat32.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
