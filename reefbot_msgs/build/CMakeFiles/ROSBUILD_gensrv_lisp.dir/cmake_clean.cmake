FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/reefbot_msgs/msg"
  "../src/reefbot_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/FindSpecies.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_FindSpecies.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
