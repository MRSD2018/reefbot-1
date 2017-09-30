FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/objdetect_msgs/msg"
  "../src/objdetect_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/objdetect_msgs/DetectObjectGridService.h"
  "../srv_gen/cpp/include/objdetect_msgs/DetectObjectService.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
