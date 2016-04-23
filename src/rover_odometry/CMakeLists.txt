cmake_minimum_required(VERSION 2.8.3)
project(rover_odometry)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    rospy
    message_generation
    tf
    nav_msgs
)

add_message_files(
  FILES
  Odometry.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
    CATKIN_DEPENDS
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# move_base
add_library(Odometry
  src/Odometry.cpp
)

add_library(Beacon
  src/Beacon.cpp
)


add_executable(rover_odometry
  src/rover_odometry.cpp
)


target_link_libraries(rover_odometry
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
#add_dependencies(move_base move_base_gencfg)
#add_dependencies(move_base geometry_msgs_gencpp)
#target_link_libraries(rover_odometry opt/ros/jade/lib/python2.7/dist-packages/tf/_tf.so opt/ros/jade/lib/python2.7/dist-packages/tf2_py/_tf2.so)
#target_link_libraries(rover_odometry opt/ros/jade/lib/libtf.so opt/ros/jade/lib/libtf2.so)

target_link_libraries(rover_odometry Beacon Odometry)
target_link_libraries(Odometry Beacon)

#set_target_properties(rover_odometry PROPERTIES OUTPUT_NAME Beacon)

#target_link_libraries(rover_odometry Odometry)
#set_target_properties(rover_odometry PROPERITES OUTPUT_NAME Odometry)

install(
    TARGETS
        Beacon
	Odometry
        rover_odometry
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

