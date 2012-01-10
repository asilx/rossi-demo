FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/feature_factory/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/feature_factory/srv/__init__.py"
  "../src/feature_factory/srv/_FeatureCalculation.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
