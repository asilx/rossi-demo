FILE(REMOVE_RECURSE
  "src/behavior_manager/srv"
  "srv_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/behavior_manager/srv/__init__.py"
  "src/behavior_manager/srv/_Action.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
