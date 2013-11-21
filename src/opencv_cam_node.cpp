#include "opencv_cam/opencv_cam.hpp"

#include <ros/ros.h>

#include <cstdlib>

int main( int argc, char *argv[] )
{
	ros::init( argc, argv, "opencv_cam" );

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv( "~" );

	opencv_cam::OpenCVCam cvcam( nh, nh_priv );

	ros::spin( );

	std::exit( EXIT_SUCCESS );
}
