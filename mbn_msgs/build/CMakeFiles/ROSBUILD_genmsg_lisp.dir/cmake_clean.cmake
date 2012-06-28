FILE(REMOVE_RECURSE
  "../src/mbn_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/MarkersIDs.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MarkersIDs.lisp"
  "../msg_gen/lisp/MarkersPoses.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MarkersPoses.lisp"
  "../msg_gen/lisp/MarkerPose.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MarkerPose.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
