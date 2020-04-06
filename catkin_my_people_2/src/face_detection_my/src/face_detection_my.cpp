#include"face_detection_my.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "face_detection_my");
	FaceDetection fd(ros::this_node::getName());
	ros::spin();
}