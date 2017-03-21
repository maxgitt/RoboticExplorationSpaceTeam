cmake_minimum_required(VERSION 2.8.3)
project(urg_node_filtered)
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



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs urg_c rosconsole nodelet dynamic_reconfigure tf laser_proc diagnostic_updater)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/URG.cfg)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES urg_c_wrapper
  CATKIN_DEPENDS roscpp sensor_msgs urg_c rosconsole nodelet dynamic_reconfigure laser_proc
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(urg_c_wrapper src/urg_c_wrapper.cpp)
target_link_libraries(urg_c_wrapper ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(urg_node src/urg_node.cpp)
add_dependencies(urg_node ${PROJECT_NAME}_gencfg)
target_link_libraries(urg_node urg_c_wrapper ${catkin_LIBRARIES})

## Declare getID tool
add_executable(getID src/getID.cpp)
target_link_libraries(getID urg_c_wrapper ${catkin_LIBRARIES})


add_executable(unit_test src/unit_test.cpp)
add_dependencies(unit_test ${PROJECT_NAME}_gencfg)
target_link_libraries(unit_test urg_c_wrapper ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS urg_c_wrapper urg_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Install script to set IP addresses
install(PROGRAMS
   scripts/set_urg_ip.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_urg_library_wrapper.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

