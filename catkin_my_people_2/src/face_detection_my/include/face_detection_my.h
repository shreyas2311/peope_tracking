#include "ros/ros.h"
#include<iostream>
#include<stdio.h>
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>

using namespace std;

class FaceDetection
{
	public:
		ros::NodeHandle nh;
		string image_raw_topic;
		string camera; 
		string camera_topic;
		string rgb;
		message_filters::Subscriber<sensor_msgs::CameraInfo> c1_info_sub_; 
		ros::SubscriberStatusCallback cloud_pub_connect_cb = boost::bind(&FaceDetection::connectCb, this);

		FaceDetection(string camera)
		{
			this->camera = camera;
			this->rgb = "rgb";
			this->image_raw_topic = "image_rect_color";
			this->camera_topic = ros::names::clean(this->camera + "/" + rgb + "/" + image_raw_topic);
		}

		~FaceDetection()
		{

		}

		


};