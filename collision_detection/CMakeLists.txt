cmake_minimum_required(VERSION 3.0.2)
project(collision_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
##generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

##add_executable(collision_detection src/collision_detection.cpp)
##target_link_libraries(collision_detection ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/regulator_cmd.py
  DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)