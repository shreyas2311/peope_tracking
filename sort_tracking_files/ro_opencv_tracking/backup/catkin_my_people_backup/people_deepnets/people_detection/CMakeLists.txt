cmake_minimum_required(VERSION 3.5.1)
project(people_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  cv_bridge
  image_geometry
  image_transport
  message_filters
  std_msgs
  people_msgs
  object_detection_msgs

)

# find_package(OpenCV 3.4.4 REQUIRED
# 	NO_MODULE
# 	PATHS /home/shreyas/OpenCV_installation/installation/OpenCV-3.4.4/share/OpenCV
# 	NO_DEFAULT_PATH
# 	)


find_package(Boost REQUIRED COMPONENTS signals system thread)



## Install project namespaced headers

generate_messages(DEPENDENCIES people_msgs object_detection_msgs)

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS
			cv_bridge
			geometry_msgs
			image_geometry
			image_transport
			message_filters
			roscpp
			roslib
			rospy
			std_msgs
			tf
			people_msgs
			object_detection_msgs
)


include_directories(
 include
	SYSTEM
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  
)


add_executable(people_detection src/people_detection.cpp)
target_link_libraries(people_detection ${OpenCV_LIBS} ${catkin_LIBRARIES})

