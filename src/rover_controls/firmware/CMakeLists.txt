cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(test
  SRCS test.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD leonardo
  PORT /dev/ttyACM0
)