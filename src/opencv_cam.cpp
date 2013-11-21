#include "opencv_cam/opencv_cam.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <webcam.h>

namespace opencv_cam
{
OpenCVCam::OpenCVCam( const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_priv )
	: nh( _nh )
	, nh_priv( _nh_priv )
	, it( nh )
	, cam_id( 0 )
	, frame_id( "camera" )
	, supported_properties( INT_MAX )
	, width( 640 )
	, height( 480 )
	, min_pub_freq( 9.0 )
	, max_pub_freq( 81.0 )
	, camera_info_man( nh, "opencv_cam" )
	, camera_pub_stats( "image_raw", diag, diagnostic_updater::FrequencyStatusParam( &min_pub_freq, &max_pub_freq, 1.0 / 3.0, 2 ), diagnostic_updater::TimeStampStatusParam( -1, 1 ) )
	, dyn_re( dyn_re_mutex, nh_priv )
	, dyn_re_cb_type( boost::bind( &OpenCVCam::DynReCB, this, _1, _2) )
{
	// Static Parameters
	nh_priv.param( "cam_id", cam_id, 0 );
	nh_priv.param( "supported_properties", supported_properties, INT_MAX );

	// Dynamic Parameters
	nh_priv.param( "frame_id", camera_info.header.frame_id, (std::string)"camera" );
	nh_priv.param( "width", width, 640 );
	nh_priv.param( "height", height, 480 );

	diag.setHardwareID( "OpenCVCam on unknown device" );
	diag.add( "OpenCV Camera Utility", this, &OpenCVCam::DiagCB );
	diag_timer = nh_priv.createWallTimer( ros::WallDuration( 1 ), &OpenCVCam::TimerCB, this );

	start( );
}

OpenCVCam::~OpenCVCam( )
{
	stop( );
}

bool OpenCVCam::start( )
{
	if( !cam.open( cam_id ) )
		goto open_fail;

	if( cam.set( CV_CAP_PROP_FRAME_WIDTH, width ) )
		goto size_fail;
	if( cam.set( CV_CAP_PROP_FRAME_HEIGHT, height ) )
		goto size_fail;

	spin_thread = boost::thread( &OpenCVCam::spin, this );

	camera_pub = it.advertiseCamera( "image_raw", 1 );

	camera_info = camera_info_man.getCameraInfo( );
	camera_info.header.frame_id = frame_id;

	diag.setHardwareIDf( "OpenCVCam on device %d", cam_id );
	dyn_re.setCallback( dyn_re_cb_type );

	return true;

	size_fail:
	cam.release( );
	open_fail:
	return false;
}

void OpenCVCam::stop( )
{
	dyn_re.clearCallback( );
	spin_thread.interrupt( );
	if( spin_thread.joinable( ) )
		spin_thread.join( );
	cam.release( );
}

void OpenCVCam::spin( )
{
	while( true )
	{
		boost::this_thread::interruption_point( );
		spinOnce( );
	}
}

void OpenCVCam::spinOnce( )
{
	static cv::Mat cv_frame;

	while( !cam.isOpened( ) || !cam.grab( ) || !camera_pub )
		usleep( 1000 );

	cam.retrieve( cv_frame );

	sensor_msgs::ImagePtr msg( new sensor_msgs::Image );
	sensor_msgs::CameraInfoPtr info_msg( new sensor_msgs::CameraInfo( camera_info ) );

	info_msg->header.stamp = ros::Time::now( );
	msg->header = info_msg->header;
	msg->height = cv_frame.size( ).height;
	msg->width = cv_frame.size( ).width;
	msg->encoding = "bgr8";
	msg->is_bigendian = 0;
	msg->step = 3 * msg->width;
	if( cv_frame.isContinuous( ) )
		msg->data.assign( cv_frame.ptr( ), cv_frame.ptr( ) + 3 * msg->width * msg->height );
	else
		msg->data.resize( 3 * msg->width * msg->height );

	camera_pub.publish( msg, info_msg );

	camera_pub_stats.tick( msg->header.stamp );
}

void OpenCVCam::TimerCB( const ros::WallTimerEvent &e )
{
	diag.update( );

	/// \note This seems to do two things:
	///       -restart the timer (otherwise the diagnostic_updater limits us and we hit 1/2 of the time)
	///       -update the timer if the diagnostic period changed

	diag_timer.setPeriod( ros::WallDuration( diag.getPeriod( ) ) );
}

void OpenCVCam::DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat )
{
	stat.add( "v4l device id", cam_id );
	stat.add( "supported properties", supported_properties );

	if( !cam.isOpened( ) )
		stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "OpenCVCam is closed" );

	static double ret;

	if( supported_properties & ( 1 << CV_CAP_PROP_POS_MSEC ) )
		if( ( ret = cam.get( CV_CAP_PROP_POS_MSEC ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_POS_MSEC", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_POS_MSEC );
	if( supported_properties & ( 1 << CV_CAP_PROP_POS_FRAMES ) )
		if( ( ret = cam.get( CV_CAP_PROP_POS_FRAMES ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_POS_FRAMES", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_POS_FRAMES );
	if( supported_properties & ( 1 << CV_CAP_PROP_POS_AVI_RATIO ) )
		if( ( ret = cam.get( CV_CAP_PROP_POS_AVI_RATIO ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_POS_AVI_RATIO", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_POS_AVI_RATIO );
	if( supported_properties & ( 1 << CV_CAP_PROP_FRAME_WIDTH ) )
		if( ( ret = cam.get( CV_CAP_PROP_FRAME_WIDTH ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_FRAME_WIDTH", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_FRAME_WIDTH );
	if( supported_properties & ( 1 << CV_CAP_PROP_FRAME_HEIGHT ) )
		if( ( ret = cam.get( CV_CAP_PROP_FRAME_HEIGHT ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_FRAME_HEIGHT", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_FRAME_HEIGHT );
	if( supported_properties & ( 1 << CV_CAP_PROP_FPS ) )
		if( ( ret = cam.get( CV_CAP_PROP_FPS ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_FPS", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_FPS );
	if( supported_properties & ( 1 << CV_CAP_PROP_FOURCC ) )
		if( ( ret = cam.get( CV_CAP_PROP_FOURCC ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_FOURCC", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_FOURCC );
	if( supported_properties & ( 1 << CV_CAP_PROP_FRAME_COUNT ) )
		if( ( ret = cam.get( CV_CAP_PROP_FRAME_COUNT ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_FRAME_COUNT", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_FRAME_COUNT );
	if( supported_properties & ( 1 << CV_CAP_PROP_FORMAT ) )
		if( ( ret = cam.get( CV_CAP_PROP_FORMAT ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_FORMAT", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_FORMAT );
	if( supported_properties & ( 1 << CV_CAP_PROP_MODE ) )
		if( ( ret = cam.get( CV_CAP_PROP_MODE ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_MODE", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_MODE );
	if( supported_properties & ( 1 << CV_CAP_PROP_BRIGHTNESS ) )
		if( ( ret = cam.get( CV_CAP_PROP_BRIGHTNESS ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_BRIGHTNESS", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_BRIGHTNESS );
	if( supported_properties & ( 1 << CV_CAP_PROP_CONTRAST ) )
		if( ( ret = cam.get( CV_CAP_PROP_CONTRAST ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_CONTRAST", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_CONTRAST );
	if( supported_properties & ( 1 << CV_CAP_PROP_SATURATION ) )
		if( ( ret = cam.get( CV_CAP_PROP_SATURATION ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_SATURATION", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_SATURATION );
	if( supported_properties & ( 1 << CV_CAP_PROP_HUE ) )
		if( ( ret = cam.get( CV_CAP_PROP_HUE ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_HUE", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_HUE );
	if( supported_properties & ( 1 << CV_CAP_PROP_GAIN ) )
		if( ( ret = cam.get( CV_CAP_PROP_GAIN ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_GAIN", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_GAIN );
	if( supported_properties & ( 1 << CV_CAP_PROP_EXPOSURE ) )
		if( ( ret = cam.get( CV_CAP_PROP_EXPOSURE ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_EXPOSURE", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_EXPOSURE );
	if( supported_properties & ( 1 << CV_CAP_PROP_CONVERT_RGB ) )
		if( ( ret = cam.get( CV_CAP_PROP_CONVERT_RGB ) ) >= 0.0 )
			stat.add( "CV_CAP_PROP_CONVERT_RGB", (bool)ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_CONVERT_RGB );
	if( supported_properties & ( 1 << CV_CAP_PROP_RECTIFICATION ) )
		if( ( ret = cam.get( CV_CAP_PROP_RECTIFICATION ) ) > 0.0 )
			stat.add( "CV_CAP_PROP_RECTIFICATION", ret );
		else
			supported_properties ^= ( 1 << CV_CAP_PROP_RECTIFICATION );


	stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "OpenCVCam is OK" );
}

void OpenCVCam::DynReCB( opencv_cam::OpenCVCamConfig &cfg, const uint32_t lvl )
{
	frame_id = cfg.frame_id;
}
}

