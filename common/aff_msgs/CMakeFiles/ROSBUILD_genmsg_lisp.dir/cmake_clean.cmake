FILE(REMOVE_RECURSE
  "src/aff_msgs/msg"
  "msg_gen"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/WorkspaceDetection.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_WorkspaceDetection.lisp"
  "msg_gen/lisp/ObjectOfInterest.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_ObjectOfInterest.lisp"
  "msg_gen/lisp/Speech.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Speech.lisp"
  "msg_gen/lisp/Features.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Features.lisp"
  "msg_gen/lisp/ExperimentState.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_ExperimentState.lisp"
  "msg_gen/lisp/ModuleStates.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_ModuleStates.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
