FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/aruco_mapping_filter/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/aruco_mapping_filter/msg/__init__.py"
  "../src/aruco_mapping_filter/msg/_MarkerArray.py"
  "../src/aruco_mapping_filter/msg/_Marker.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
