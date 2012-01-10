FILE(REMOVE_RECURSE
  "src/aff_msgs/msg"
  "msg_gen"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/aff_msgs/msg/__init__.py"
  "src/aff_msgs/msg/_WorkspaceDetection.py"
  "src/aff_msgs/msg/_ObjectOfInterest.py"
  "src/aff_msgs/msg/_Speech.py"
  "src/aff_msgs/msg/_Features.py"
  "src/aff_msgs/msg/_ExperimentState.py"
  "src/aff_msgs/msg/_ModuleStates.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
