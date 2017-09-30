FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/objdetect_msgs/msg"
  "../src/objdetect_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/objdetect_msgs/Grid.h"
  "../msg_gen/cpp/include/objdetect_msgs/DetectGridScores.h"
  "../msg_gen/cpp/include/objdetect_msgs/Detection.h"
  "../msg_gen/cpp/include/objdetect_msgs/DetectObject.h"
  "../msg_gen/cpp/include/objdetect_msgs/DetectionArray.h"
  "../msg_gen/cpp/include/objdetect_msgs/Mask.h"
  "../msg_gen/cpp/include/objdetect_msgs/DetectObjectGrid.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
