FILE(REMOVE_RECURSE
  "src/tabletop_2D_segmentation/srv"
  "srv_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/Perception2D.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_Perception2D.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
