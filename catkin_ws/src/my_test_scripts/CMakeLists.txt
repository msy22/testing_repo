cmake_minimum_required(VERSION 2.8.3)
project(my_test_scripts)

## Required in order to use C++11 stuff, however some aspects of PCL don't work
## with C++11, and will cause segmentation faults with no warning at runtime.
##add_definitions(-std=c++11)

set(Boost_DEBUG 1)
#set(BOOST_ROOT /usr/lib/boost_1_65_1)
#set(BOOST_INCLUDEDIR /usr/lib/boost_1_65_1)
#set(BOOST_LIBRARYDIR /usr/lib/boost_1_65_1/stage/lib)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  roslaunch
)

#find_package(Boost REQUIRED COMPONENTS
#  algorithm
#  core
#)

#find_package(Boost REQUIRED 1.65.1  COMPONENTS
#  algorithm
#  core
#)

## Declare a catkin package
## this is necessary otherwise the .cpp file won't be callable
catkin_package()

#${Boost_INCLUDE_DIRS}
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINTIONS})

add_executable(testing_script src/testing_script.cpp)
target_link_libraries(testing_script ${PCL_LIBRARIES} ${catkin_LIBRARIES})

