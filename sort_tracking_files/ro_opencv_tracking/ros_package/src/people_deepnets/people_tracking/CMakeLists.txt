cmake_minimum_required(VERSION 2.8.3)
project(people_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  object_detection_msgs
  people_msgs
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS signals system thread)

generate_messages(DEPENDENCIES people_msgs object_detection_msgs)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES people_tracking
 CATKIN_DEPENDS cv_bridge image_geometry roslib image_transport message_filters object_detection_msgs people_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(people_tracking_kcf src/people_tracking_kcf.cpp)
target_link_libraries(people_tracking_kcf ${OpenCV_LIBS} ${catkin_LIBRARIES})
