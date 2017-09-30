FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/reefbot_msgs/msg"
  "../src/reefbot_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/reefbot_msgs/SpeciesIDResponse.h"
  "../msg_gen/cpp/include/reefbot_msgs/SpeciesScore.h"
  "../msg_gen/cpp/include/reefbot_msgs/SingleSpeciesId.h"
  "../msg_gen/cpp/include/reefbot_msgs/CameraHealth.h"
  "../msg_gen/cpp/include/reefbot_msgs/RobotHealth.h"
  "../msg_gen/cpp/include/reefbot_msgs/ImageCaptured.h"
  "../msg_gen/cpp/include/reefbot_msgs/LogVideo.h"
  "../msg_gen/cpp/include/reefbot_msgs/VideoStream.h"
  "../msg_gen/cpp/include/reefbot_msgs/UserSpeciesSelection.h"
  "../msg_gen/cpp/include/reefbot_msgs/RobotStatus.h"
  "../msg_gen/cpp/include/reefbot_msgs/AddSpeciesLabel.h"
  "../msg_gen/cpp/include/reefbot_msgs/SpeciesIDRequest.h"
  "../msg_gen/cpp/include/reefbot_msgs/CameraPower.h"
  "../msg_gen/cpp/include/reefbot_msgs/ImageRegion.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
