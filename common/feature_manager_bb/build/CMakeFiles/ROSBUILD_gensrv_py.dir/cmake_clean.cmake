FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/feature_manager/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/feature_manager/srv/__init__.py"
  "../src/feature_manager/srv/_Perception.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
