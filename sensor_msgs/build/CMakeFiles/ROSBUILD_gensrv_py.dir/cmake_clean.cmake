FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sensor_msgs/msg"
  "../src/sensor_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/sensor_msgs/srv/__init__.py"
  "../src/sensor_msgs/srv/_SetCameraInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
