FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/objdetect_msgs/msg"
  "../src/objdetect_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/objdetect_msgs/msg/__init__.py"
  "../src/objdetect_msgs/msg/_Grid.py"
  "../src/objdetect_msgs/msg/_DetectGridScores.py"
  "../src/objdetect_msgs/msg/_Detection.py"
  "../src/objdetect_msgs/msg/_DetectObject.py"
  "../src/objdetect_msgs/msg/_DetectionArray.py"
  "../src/objdetect_msgs/msg/_Mask.py"
  "../src/objdetect_msgs/msg/_DetectObjectGrid.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
