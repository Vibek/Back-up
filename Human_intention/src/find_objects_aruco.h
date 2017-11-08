#ifndef __OBJECTS_FINDER_ARUCO_H__
#define __OBJECTS_FINDER_ARUCO_H__

//Standard ROS headers
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>

//Standard c++ headers
#include <iostream>
#include <fstream>

//Aruco headers
#include "Aruco/aruco.h"
#include "Aruco/cameraparameters.h"
#include "Aruco/cvdrawingutils.h"
#include "Aruco/arucofidmarkers.h"
#include "Human_intention/Marker.h"
#include "Human_intention/MarkerArray.h"

//OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "find_objects_base.h"



class  ObjectFinderAruco : public ObjectsFinderBase
{

private:
cv::Mat I;                                       //OpeCV image

    //Calibration parameters
    std::string filename;                            //calibration file path
    cv::Mat *intrinsics;                             //camera intrinsics
    cv::Mat *distortion_coeff;                       //camera distortion coeffs
    cv::Size *image_size;                            //image_size

    //ROS messaging
    ros::Publisher marker_pub;                       //marker visualization

    //Chessboard detection variables
    cv::Size board_size;                             //widht and height of calibration grid
    double square_size;                               //size of grid square

    std::vector <cv::Point3f> chessboard3D_points;   //chessboard 3D points in own reference frame
    std::vector<cv::Point2f>  chessboard2D_points;   //chessboard 2D points in image

    std::vector <cv::Point3d> ref_frame_points;      //reference xyz frame points
    std::vector<cv::Point2d>  image_frame_points;    //reference xyz frame points in image

    //Marker parameters
    double marker_size;                               //marker geometry
    aruco::CameraParameters aruco_calib_params;      //Camera parameters for aruco lib

    //TF frame
    std::string camera_frame;                        //parrent camera frame name
    image_transport::Subscriber subImage;

private:
	bool markers_find_pattern(cv::Mat input_image,cv::Mat output_image);
	bool load_calibration_file(std::string filename);

public:
	~ObjectFinderAruco();
	virtual void init(int width, int height);
	virtual cv::Mat detectObjects(cv::Mat frame);

	//void processPoint(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
};

#endif
