cmake_minimum_required(VERSION 2.8.3)

link_directories(/home/gabriel/sketchbook/libraries)
link_directories(/home/gabriel/sketchbook/libraries/ros_lib/rover_controls)
include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

set(romeo_LIBRARY Wire)
set(romeo_LIBRARY FreeSixIMU)
set(romeo_LIBRARY rover_controls)

generate_arduino_firmware(romeo
	SRCS romeo.cpp ${ROS_LIB_DIR}/time.cpp
	BOARD leonardo #mega2560
	PROGRAMMER avrispmkii
	PORT /dev/romeo
)