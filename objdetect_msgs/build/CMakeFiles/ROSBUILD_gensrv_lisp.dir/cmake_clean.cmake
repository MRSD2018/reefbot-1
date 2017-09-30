FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/objdetect_msgs/msg"
  "../src/objdetect_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/DetectObjectGridService.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_DetectObjectGridService.lisp"
  "../srv_gen/lisp/DetectObjectService.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_DetectObjectService.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
