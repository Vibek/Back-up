FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/aruco_mapping_filter/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/MarkerArray.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MarkerArray.lisp"
  "../msg_gen/lisp/Marker.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Marker.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
