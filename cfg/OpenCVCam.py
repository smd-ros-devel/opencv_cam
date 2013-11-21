#!/usr/bin/env python

PACKAGE='opencv_cam'

from driver_base.msg import SensorLevels
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator( )
global_params = gen.add_group( "Global Parameters" )

global_params.add( "frame_id", str_t, SensorLevels.RECONFIGURE_RUNNING, "Frame ID for published images", "camera" )

exit( gen.generate( PACKAGE, "opencv_cam", "OpenCVCam" ) )
