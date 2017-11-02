# aruco_marker_tracking

Improving performance of [aruco_tracking](http://wiki.ros.org/aruco_mapping) package for ROS groovy using rosmake

Author: Vibekananda Dutta


## ROS API:

**Subsrcibed Topics**: /camera/image_raw*

**Published Topics**: /camera/image_raw_filtered*

**Parameters**:

Name          | Type         | Range       | Comment                  |
------------- | -------------| --------------------| -------------------------|
threshold | int | 0-255 | 0 means original image |


