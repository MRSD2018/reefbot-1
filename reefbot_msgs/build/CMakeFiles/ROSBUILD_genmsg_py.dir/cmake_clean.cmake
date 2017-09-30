FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/reefbot_msgs/msg"
  "../src/reefbot_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/reefbot_msgs/msg/__init__.py"
  "../src/reefbot_msgs/msg/_SpeciesIDResponse.py"
  "../src/reefbot_msgs/msg/_SpeciesScore.py"
  "../src/reefbot_msgs/msg/_SingleSpeciesId.py"
  "../src/reefbot_msgs/msg/_CameraHealth.py"
  "../src/reefbot_msgs/msg/_RobotHealth.py"
  "../src/reefbot_msgs/msg/_ImageCaptured.py"
  "../src/reefbot_msgs/msg/_LogVideo.py"
  "../src/reefbot_msgs/msg/_VideoStream.py"
  "../src/reefbot_msgs/msg/_UserSpeciesSelection.py"
  "../src/reefbot_msgs/msg/_RobotStatus.py"
  "../src/reefbot_msgs/msg/_AddSpeciesLabel.py"
  "../src/reefbot_msgs/msg/_SpeciesIDRequest.py"
  "../src/reefbot_msgs/msg/_CameraPower.py"
  "../src/reefbot_msgs/msg/_ImageRegion.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
