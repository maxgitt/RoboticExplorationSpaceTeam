cmake_minimum_required(VERSION 2.8.3)
project(rover_excavation)
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
    message_generation
)

add_service_files(
  FILES
  finish_dig.srv
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
add_library(dig_control
  src/dig_control.cpp
)


add_executable(rover_excavation
  src/rover_excavation.cpp
)


target_link_libraries(rover_excavation
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
#add_dependencies(move_base move_base_gencfg)
#add_dependencies(move_base geometry_msgs_gencpp)


target_link_libraries(rover_excavation dig_control)
#target_link_libraries(rover_excavation)
#set_target_properties(rover_odometry PROPERTIES OUTPUT_NAME Beacon)

#target_link_libraries(rover_odometry Odometry)
#set_target_properties(rover_odometry PROPERITES OUTPUT_NAME Odometry)


## Build service client and server
# %Tag(SRVCLIENT)%
#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client beginner_tutorials_gencpp)


install(
    TARGETS
	dig_control
        rover_excavation
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

