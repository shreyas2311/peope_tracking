cmake_minimum_required(VERSION 2.8.3)
project(face_detection_my)

find_package(catkin REQUIRED COMPONENTS
   actionlib
        actionlib_msgs
        cv_bridge
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_generation
        people_msgs
        rosbag
        roscpp
        roslib
        rospy
        sensor_msgs
        std_msgs
        std_srvs
        stereo_msgs
        tf
)


catkin_package(
  INCLUDE_DIRS include

  DEPENDS
  		cv_bridge
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_runtime
        roscpp
        roslib
        rospy
        sensor_msgs
        std_msgs
        stereo_msgs
        tf

)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)


add_executable(
	face_detection_my
			src/face_detection_my.cpp
)

 target_link_libraries(face_detection_my
   ${catkin_LIBRARIES}
 )

 add_dependencies(face_detection_my ${catkin_EXPORTED_TARGETS})


