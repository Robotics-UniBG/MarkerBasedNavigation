FILE(REMOVE_RECURSE
  "../src/mbn_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/mbn_msgs/MarkersIDs.h"
  "../msg_gen/cpp/include/mbn_msgs/MarkersPoses.h"
  "../msg_gen/cpp/include/mbn_msgs/MarkerPose.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
