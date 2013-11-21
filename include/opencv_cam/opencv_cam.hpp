#include "opencv_cam/OpenCVCamConfig.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace opencv_cam
{
class OpenCVCam
{
public:
	OpenCVCam( const ros::NodeHandle &_nh = ros::NodeHandle( ),
		const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ) );
	~OpenCVCam( );
protected:
	bool start( );
	void stop( );
	void spin( );
	void spinOnce( );
private:
	void TimerCB( const ros::WallTimerEvent &e );
	void DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat );
	void DynReCB( opencv_cam::OpenCVCamConfig &cfg, const uint32_t lvl );

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;

	int cam_id;
	std::string frame_id;
	int supported_properties;
	int width;
	int height;
	double min_pub_freq;
	double max_pub_freq;

	sensor_msgs::CameraInfo camera_info;
	image_transport::ImageTransport it;
	camera_info_manager::CameraInfoManager camera_info_man;
	image_transport::CameraPublisher camera_pub;
	ros::WallTimer diag_timer;
	diagnostic_updater::Updater diag;
	diagnostic_updater::TopicDiagnostic camera_pub_stats;
	boost::recursive_mutex dyn_re_mutex;
	dynamic_reconfigure::Server<opencv_cam::OpenCVCamConfig> dyn_re;
	dynamic_reconfigure::Server<opencv_cam::OpenCVCamConfig>::CallbackType dyn_re_cb_type;

	cv::VideoCapture cam;

	boost::thread spin_thread;
};
}
