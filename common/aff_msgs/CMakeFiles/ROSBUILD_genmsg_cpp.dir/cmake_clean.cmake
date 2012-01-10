FILE(REMOVE_RECURSE
  "src/aff_msgs/msg"
  "msg_gen"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/aff_msgs/WorkspaceDetection.h"
  "msg_gen/cpp/include/aff_msgs/ObjectOfInterest.h"
  "msg_gen/cpp/include/aff_msgs/Speech.h"
  "msg_gen/cpp/include/aff_msgs/Features.h"
  "msg_gen/cpp/include/aff_msgs/ExperimentState.h"
  "msg_gen/cpp/include/aff_msgs/ModuleStates.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
