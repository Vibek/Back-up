#include <ros/ros.h>
#include "find_objects_aruco.h"


void  ObjectFinderAruco::init(int width, int height)
{

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    nh.param("calibration_file", filename, std::string("empty"));
    nh.param("marker_size", marker_size, 0.1);  //Default Marker size in cm
    nh.param("square_size", square_size, 2.75); //Chessboard square size

    load_calibration_file(filename);                //Load camera calibration data
}

ObjectFinderAruco::~ObjectFinderAruco()
{
    if (intrinsics) delete intrinsics;
    if (distortion_coeff) delete distortion_coeff;
    if (image_size) delete image_size;
}

cv::Mat  ObjectFinderAruco::detectObjects(cv::Mat fr)
{
//ROS Image to Mat structure
  /*cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(fr, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("red_ball_detection::cv_bridge_exception %s", e.what());
    return;
  }
  I = cv_ptr->image;*/

  I = fr;

  //============================================
  //Choose Chessboard or Marker detection
  //bool found = chessboard_find_pattern(I,I);
  bool found = markers_find_pattern(I,I);
  //============================================

   return I;
   //cv::waitKey(10);
}

bool ObjectFinderAruco::markers_find_pattern(cv::Mat input_image,cv::Mat output_image)
{
  aruco::MarkerDetector MDetector;
  std::vector<aruco::Marker> markers; 
  static tf::TransformBroadcaster br;

  try {
    MDetector.detect(input_image, markers, aruco_calib_params, marker_size);
  }
  catch(cv::Exception e)
  {
    std::cout << "ObjectFinderAruco::MDetector exception: " << e.what();
  }

  for(size_t i = 0; i < markers.size(); i++)
  {
    break;
    int current_marker_id = markers[i].id;
    //Draw marker convex, ID, cube and axis
    markers[i].draw(output_image, cv::Scalar(0,0,255), 2);
    aruco::CvDrawingUtils::draw3dCube(output_image, markers[i], aruco_calib_params);
    aruco::CvDrawingUtils::draw3dAxis(output_image, markers[i], aruco_calib_params);


#if 0    //Transform marker pose to TFformat
    tf::Transform object_transform = arucoMarker2Tf(markers[i]);
    
    //Marker ID to string
    std::stringstream marker_id_string;
    marker_id_string << "marker_" << current_marker_id;
    //Publish Current Marker TF
    br.sendTransform(tf::StampedTransform(object_transform, ros::Time::now(), camera_frame, marker_id_string.str()));

    //=================================================
    //Publish Current Marker to RViz
    //=================================================
    geometry_msgs::Pose marker_pose_data;
    
    const tf::Vector3 marker_origin = object_transform.getOrigin();
    marker_pose_data.position.x = marker_origin.getX();
    marker_pose_data.position.y = marker_origin.getY();
    marker_pose_data.position.z = marker_origin.getZ();
    
    tf::Quaternion marker_quaternion = object_transform.getRotation();
    marker_pose_data.orientation.x = marker_quaternion.getX();
    marker_pose_data.orientation.y = marker_quaternion.getY();
    marker_pose_data.orientation.z = marker_quaternion.getZ();
    marker_pose_data.orientation.w = marker_quaternion.getW();
 
    publish_marker(marker_pose_data, current_marker_id);
#endif
  }
  
#if 0
  //Display camera frame
  tf::Vector3 camera_translation(0,0,0);
  tf::Matrix3x3 camera_rotation(1,0,0,
                                0,1,0,
                                0,0,1);
 
  tf::Transform camera_transform(camera_rotation, camera_translation);
  br.sendTransform(tf::StampedTransform(camera_transform, ros::Time::now(), camera_frame, "camera"));
#endif
}

bool ObjectFinderAruco::load_calibration_file(std::string filename)
{
  std::cout << "Reading calibration file from: " << filename << std::endl;
  try
  {
      //Searching camera matrix and distortion in calibration textfile
      //# oST version 5.0 parameters
      string camera_matrix_str("camera matrix");
      string distortion_str("distortion");

      ifstream file;
      file.open(filename.c_str());

      intrinsics = new(cv::Mat)(3,3,CV_64F);
      distortion_coeff = new(cv::Mat)(5,1,CV_64F);
      image_size = new(cv::Size);

      std::string line;
      int line_counter = 0;
      while(getline(file, line))
      {
          if(line == camera_matrix_str)
          {
              for(size_t i = 0; i < 3; i++)
                  for(size_t j = 0; j < 3; j++)
                     file >> intrinsics->at<double>(i,j);

		  std::cout << "Intrinsics:" << std::endl << *intrinsics << std::endl;
          }
          if(line == distortion_str)
          {
              for(size_t i = 0; i < 5; i++)
                 file >> distortion_coeff->at<double>(i,0);

              std::cout << "Distortion: " << *distortion_coeff << std::endl;
          }
          line_counter++;
      }

      aruco_calib_params.setParams(*intrinsics, *distortion_coeff, *image_size);

      if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
          ROS_INFO_STREAM("Calibration file loaded successfully");
      else
	  ROS_WARN("WARNING: Suspicious calibration data");
	
  }
  catch(int e)
  {
     std::cout << "An exception n." << e << "occured";
  }
}




