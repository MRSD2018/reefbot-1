FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/reefbot_msgs/msg"
  "../src/reefbot_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/reefbot_msgs/FindSpecies.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
