FILE(REMOVE_RECURSE
  "../src/pr2_learning_affordances/msg"
  "../src/pr2_learning_affordances/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/pr2_learning_affordances/msg/__init__.py"
  "../src/pr2_learning_affordances/msg/_DataCollect.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
