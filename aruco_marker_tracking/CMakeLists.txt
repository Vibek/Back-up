cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

#uncomment if you have defined services
#rosbuild_gensrv()

find_package(OpenCV REQUIRED)

rosbuild_add_library(aruco src/arucofidmarkers.cpp src/cvdrawingutils.cpp src/cameraparameters.cpp src/board.cpp src/marker.cpp src/boarddetector.cpp src/markerdetector.cpp)

target_link_libraries(aruco ${OpenCV_LIBS})

rosbuild_add_executable(filter src/aruco_marker_detection.cpp)

target_link_libraries(filter aruco ${OpenCV_LIBS})
