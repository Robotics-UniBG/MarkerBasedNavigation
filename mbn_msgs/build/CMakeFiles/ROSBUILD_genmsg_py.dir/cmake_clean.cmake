FILE(REMOVE_RECURSE
  "../src/mbn_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/mbn_msgs/msg/__init__.py"
  "../src/mbn_msgs/msg/_MarkersIDs.py"
  "../src/mbn_msgs/msg/_MarkersPoses.py"
  "../src/mbn_msgs/msg/_MarkerPose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
