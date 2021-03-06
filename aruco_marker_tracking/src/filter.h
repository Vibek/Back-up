
#ifndef FILTER_H
#define FILTER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/** \brief Image filtering for Aruco */
//namespace image_filtering_for_aruco
//{
  
/** \brief Client class for Aruco image filtering */
class Filter {
public:
  	
	/** \brief Construct a client for EZN64 USB control*/    
  	   Filter();
	  ~Filter();
 
private:

     /** \brief Callback function for filtering */    
   	void ImageCallback(const sensor_msgs::ImageConstPtr &original_image);
     /** \brief User threshold loaded from the launch file */
    	int user_threshold;
    
    /** \brief CV bridge to convert std_msgs::Image to OpenCV format*/
    	cv_bridge::CvImagePtr cv_ptr;
   
    /** \brief Original image */
    	cv::Mat I;
    
    /** \brief Filtered image */
    	cv::Mat I_filtered;
    
    /** \brief Filtered image publisher */
    	image_transport::Publisher filtered_image_pub;
	image_transport::Subscriber subImage;

}; //Filter
//}  //image_filtering_for_aruco

#endif // FILTER_H
