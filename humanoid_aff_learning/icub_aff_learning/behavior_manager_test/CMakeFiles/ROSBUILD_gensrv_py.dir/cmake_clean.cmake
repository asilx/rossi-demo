FILE(REMOVE_RECURSE
  "srv_gen"
  "src/behavior_manager_test/srv"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/behavior_manager_test/srv/__init__.py"
  "src/behavior_manager_test/srv/_Action.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
