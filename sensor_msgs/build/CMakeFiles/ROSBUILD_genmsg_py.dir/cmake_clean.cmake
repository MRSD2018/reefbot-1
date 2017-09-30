FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sensor_msgs/msg"
  "../src/sensor_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/sensor_msgs/msg/__init__.py"
  "../src/sensor_msgs/msg/_CompressedImage.py"
  "../src/sensor_msgs/msg/_NavSatStatus.py"
  "../src/sensor_msgs/msg/_MatND.py"
  "../src/sensor_msgs/msg/_CameraInfo.py"
  "../src/sensor_msgs/msg/_LaserScan.py"
  "../src/sensor_msgs/msg/_Imu.py"
  "../src/sensor_msgs/msg/_PointCloud2.py"
  "../src/sensor_msgs/msg/_JoyFeedback.py"
  "../src/sensor_msgs/msg/_JoyFeedbackArray.py"
  "../src/sensor_msgs/msg/_PointField.py"
  "../src/sensor_msgs/msg/_RegionOfInterest.py"
  "../src/sensor_msgs/msg/_JointState.py"
  "../src/sensor_msgs/msg/_NavSatFix.py"
  "../src/sensor_msgs/msg/_PointCloud.py"
  "../src/sensor_msgs/msg/_Joy.py"
  "../src/sensor_msgs/msg/_Range.py"
  "../src/sensor_msgs/msg/_Image.py"
  "../src/sensor_msgs/msg/_ChannelFloat32.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
