#include "people_tracking/people_tracking_kcf.h"

namespace People_Tracker
{
	unsigned int People_tracker_KCF::id_seed = 0;
	People_tracker_KCF::People_tracker_KCF(string name):it_(nh_)
	{
		is_tracking = false;
		ROS_INFO("Created people tracker kcf");
		camera_name = nh_.resolveName("camera");
		image_topic_name = nh_.resolveName("image_topic");
		depth_topic_name = nh_.resolveName("depth_topic");

		rgb_ns = nh_.resolveName("rgb_ns");
		depth_ns = nh_.resolveName("depth_ns");
		camera_info = nh_.resolveName("camera_info");
		bbox_ns = nh_.resolveName("bbox_topic_ns");
		bbox_topic_name = nh_.resolveName("bbox_topic");
		bbox_topic = ros::names::clean(bbox_ns + "/" + bbox_topic_name);
		image_topic = ros::names::clean(camera_name + "/" + rgb_ns + "/" + image_topic_name);
		depth_topic = ros::names::clean(camera_name + "/" + depth_ns + "/" + depth_topic_name);

		camera_rgb_info_topic = ros::names::clean(camera_name + "/" + rgb_ns + "/" + camera_info);
		camera_depth_info_topic = ros::names::clean(camera_name + "/" + depth_ns + "/" + camera_info);
		

		// approximate_bbox_sync.reset(new ApproximateBoundingBoxSync(ApproximateBoundingBoxPolicy(queue_size), image_sub_, c1_info_sub_, bbox_sub));
		// approximate_bbox_sync->registerCallback(boost::bind(&People_tracker_KCF::image_and_bbox_cb, this, _1, _2, _3));
		
		// approximate_bbox_sync_test.reset(new ApproximateBoundingBoxSyncTest(ApproximateBoundingBoxPolicyTest(queue_size), image_sub_, c1_info_sub_));
		// approximate_bbox_sync_test->registerCallback(boost::bind(&People_tracker_KCF::image_and_bbox_cb, this, _1, _2));
		/***testing KCF code ****/		
		string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
		string trackerType = trackerTypes[2];
		// Ptr<Tracker> tracker;
		// tracker = TrackerKCF::create();
		// Rect2d bbox(124, 32, 200, 280);
		// tracker_list.push_back(tracker);
		/***********************/	
		ROS_INFO("Please subscribe to people_tracker_measurements.");
		ros::SubscriberStatusCallback position_publisher_connect_callback = boost::bind(&People_tracker_KCF::connect_callback, this);
		people_tracker_measurements_pub = nh_.advertise<people_msgs::PositionMeasurementArray>("people_tracker/person_tracker_measurement", 10, position_publisher_connect_callback, position_publisher_connect_callback);
		

		// approximate_bbox_sync_test_2.reset(new ApproximateBoundingBoxSyncTest(ApproximateBoundingBoxPolicyTest(queue_size), image_sub_, c1_info_sub_));
		// approximate_bbox_sync_test_2->registerCallback(boost::bind(&People_tracker_KCF::image_and_bbox_cb_test_2, this, _1, _2));
		
		// approximate_depth_sync_.reset(new ApproximateDepthSync(ApproximateDepthPolicy(queue_size), image_sub_, depth_image_sub_,c1_info_sub_, c2_info_sub_));
		// approximate_depth_sync_->registerCallback(boost::bind(&People_tracker_KCF::image_and_depth_callback, this, _1, _2, _3, _4));
				
		approximate_bbox_sync.reset(new ApproximateBoundingBoxSync(ApproximateBoundingBoxSync(queue_size), image_sub_, depth_image_sub_, c1_info_sub_, c2_info_sub_, bbox_sub));
		approximate_bbox_sync->registerCallback(boost::bind(&People_tracker_KCF::image_depth_bbox_callback, this, _1, _2, _3, _4, _5));
		
		ros::spin();
	}

	// unsigned int People_tracker_KCF::get_new_ID()
	// {
	// 	// return 1;
	// 	People_tracker_KCF::id_seed = People_tracker_KCF::id_seed +1;
	// 	return People_tracker_KCF::id_seed ;
	// }

	void People_tracker_KCF::connect_callback()
	{
		if(people_tracker_measurements_pub.getNumSubscribers() == 0)
		{
			ROS_INFO("Please subscribe to people detectors outbound topics");
			image_sub_.unsubscribe();
			depth_image_sub_.unsubscribe();
			c1_info_sub_.unsubscribe();
			c2_info_sub_.unsubscribe();	
			bbox_sub.unsubscribe();
		}
		else
		{
			ROS_INFO("%s", bbox_topic.c_str());
			image_sub_.subscribe(it_, image_topic, 3);
			depth_image_sub_.subscribe(it_, depth_topic, 3);
			c1_info_sub_.subscribe(nh_, camera_rgb_info_topic, 3);
			bbox_sub.subscribe(nh_, bbox_topic, 3);
			c2_info_sub_.subscribe(nh_, camera_depth_info_topic, 3);
		}
	}

	// void People_tracker_KCF::image_and_depth_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info)
	// {
	// 	// ROS_INFO("hi");
	// 	cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");	
	// 	Mat image_display = cv_image_ptr->image.clone();
	// 	Rect2d bbox(0, 0, 0, 0);
	// 	unordered_map<unsigned int, Ptr<TrackerKCF>>::iterator it = tracker_list.begin();
	// 	cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image);
	// 	cv::Mat depth_32fc1 = cv_depth_ptr->image;
	// 	if (depth_image->encoding != "32FC1")
	// 	{
	// 			cv_depth_ptr->image.convertTo(depth_32fc1, CV_32FC1, 0.001);
	// 	}
	// 	cam_model_.fromCameraInfo(c1_info, c2_info);

	// 	while(it != tracker_list.end())
	// 	{
	// 		// ROS_INFO("hi %d", tracker_list.size());

	// 		if(it->second->update(cv_image_ptr->image, bbox))
	// 		{

	// 			// // rectangle(image_display, bbox, Scalar( 255, 0, 0 ), 2, 1 ); 
	// 			// // ROS_INFO("x %f", bbox.x);
	// 			Point2d center2d = Point2d(bbox.x + (bbox.width/2.0),
	// 									   bbox.y + (bbox.height/2.0)  );

	// 			// ROS_INFO("width %f", bbox.width);
	// 			// ROS_INFO("height %f", bbox.height);

	// 			Mat depth_roi(  depth_32fc1, Rect( floor(bbox.x + 0.25*bbox.width),
	// 											   floor(bbox.y + 0.25*bbox.height),
	// 											   floor(bbox.x + 0.75*bbox.width) - floor(bbox.x + 0.25*bbox.width) + 1,
	// 											   floor(bbox.y + 0.75*bbox.height) - floor(bbox.y + 0.25*bbox.height) + 1 )  );

	// 			std::vector<float> depths;

	// 			for (int i =0; i<depth_roi.rows; i++)
	// 			{
	// 				float *dptr = depth_roi.ptr<float>(i);
	// 				for(int j = 0; j<depth_roi.cols; j++)
	// 				{
	// 					if(dptr[j] == dptr[j]) //To eliminate nans
	// 					{
	// 						depths.push_back(dptr[j]);	
	// 					}
	// 				}
	// 			}

	// 			std::vector<float>::iterator dbegin = depths.begin();
	// 			std::vector<float>::iterator dend = depths.end();
	// 			if (depths.size() > 0)
	// 			{
	// 				std::sort(dbegin, dend);
	// 				double avg_d = depths[floor(depths.size() / 2.0)];
	// 				// each_person.width3d = fabs((cam_model->left()).getDeltaX(each_person.bounding_box.width, avg_d));
					
	// 				Point3d center3d = (cam_model_.left()).projectPixelTo3dRay(center2d);
	// 				center3d = (avg_d / center3d.z) * center3d;
	// 				tracked_points[it->first]= center3d; 
	// 			}



	// 		}
	// 		else
	// 		{

	// 		}

	// 		// if(ok)
	// 		// {
	// 		// 	ROS_INFO("x %f", bbox.x);
	// 		// 	// ROS_INFO("y %f", bbox.y);
	// 		// 	// ROS_INFO("width %f", bbox.width);
	// 		// 	// ROS_INFO("height %f", bbox.height);
	// 			// rectangle(image_display, bbox, Scalar( 255, 0, 0 ), 2, 1 ); 
	// 		// }
	// 		it++;
	// 	}

	// 	imshow("Tracking", image_display);
	// 	waitKey(1);

	// }

	// void People_tracker_KCF::image_and_bbox_cb(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::CameraInfo::ConstPtr &c1_info, const object_detection_msgs::BoundingBoxMeasurementsArray::ConstPtr &bbox_measurements_array )
	// {
	// 	ROS_INFO("calling image and bbox callback");

	// }

	// void People_tracker_KCF::image_and_bbox_cb_test(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::CameraInfo::ConstPtr &c1_info)
	// {
	// 	if(!done)
	// 	{
	// 		Rect2d bbox(124, 32, 200, 280); 
	// 		Ptr<TrackerKCF> tracker = TrackerKCF::create();;


	// 		// // ROS_INFO("calling image and bbox callbacktest");
	// 		cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");

	// 		tracker->init(cv_image_ptr->image, bbox);
	// 		tracker_list.push_back(tracker);
	// 		done  = true;
	// 		ROS_INFO("DONE");
	// 	}
	// 	// track_people(cv_image_ptr->image);
	// }

	// void People_tracker_KCF::image_and_bbox_cb_test_2(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::CameraInfo::ConstPtr &c1_info)
	// {
	// 	cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");
	// 	Rect2d bbox2(417, 178, 100, 130);   
		
	// 	cv::Mat image_copy = (cv_image_ptr->image).clone();
	// 	for(int i=0; i < tracker_list.size(); i++)
	// 	{
 //   			tracker_list[i]->update(image_copy, bbox2);	
	// 	}

	// 	rectangle(image_copy, bbox2, Scalar(0, 255, 0), 2);

	// 	imshow("Frame", image_copy);
	// 	waitKey(1);





	// 	// ROS_INFO("HI");
	// 	// Rect2d bbox(124, 32, 200, 280); 
	// 	// Ptr<TrackerKCF> tracker = TrackerKCF::create();;


	// 	// // // ROS_INFO("calling image and bbox callbacktest");
	// 	// cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");

	// 	// tracker->init(cv_image_ptr->image, bbox);
	// 	// tracker_list.push_back(tracker);
	// 	// track_people(cv_image_ptr->image);
	// }

	// void People_tracker_KCF::track_people(const cv::Mat image)
	// {
	// 	cv::Mat image_copy = image.clone();
	// 	Rect2d bbox(124, 32, 200, 280); 		
	// 	rectangle(image_copy, bbox, Scalar( 255, 0, 0 ), 2, 1 ); 
	// 	tracker->init(image, bbox);
	// }

	void People_tracker_KCF::image_depth_bbox_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info, const object_detection_msgs::BoundingBoxMeasurementsArray::ConstPtr &bbox_measurements_array )
	{
		// ROS_INFO("Here");
		cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");

		if(!is_tracking)
		{
			object_detection_msgs::BoundingBox min_bbox = bbox_measurements_array->bounding_box_array[0];
			float min_dist =  FLT_MAX;

			for(int i = 0; i < bbox_measurements_array->bounding_box_array.size(); i ++ )
			{

				object_detection_msgs::BoundingBox present_bbox = bbox_measurements_array->bounding_box_array[i];
				
				geometry_msgs::Point *present_coor = &present_bbox.center;
				
				float distance_val = sqrt(  pow(present_coor->x,2) + pow(present_coor->y,2) + pow(present_coor->z,2) );

				if(distance_val < min_dist)
				{
					min_dist = distance_val;
					min_bbox = bbox_measurements_array->bounding_box_array[i];
				}
			}

			is_tracking = true;
			person_tracker = TrackerKCF::create();
			Rect2d bbox(min_bbox.bbox_coors.x, min_bbox.bbox_coors.y, min_bbox.bbox_width, min_bbox.bbox_height); 		
			person_tracker->init(cv_image_ptr->image, bbox);
			person_bbox = bbox;
		}
		else
		{
			// Rect2d bbox(0,0,0,0);
			if(person_tracker->update(cv_image_ptr->image, person_bbox))
			{

				Point2d center2d = Point2d(person_bbox.x + (person_bbox.width/2.0),
										 person_bbox.y + (person_bbox.height/2.0)  );


				cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image);
				cv::Mat depth_32fc1 = cv_depth_ptr->image;

				if (depth_image->encoding != "32FC1")
				{
						cv_depth_ptr->image.convertTo(depth_32fc1, CV_32FC1, 0.001);
				}
				cam_model_.fromCameraInfo(c1_info, c2_info);

				Mat depth_roi(  depth_32fc1, Rect( floor(person_bbox.x + 0.25*person_bbox.width),
										   floor(person_bbox.y + 0.25*person_bbox.height),
										   floor(person_bbox.x + 0.75*person_bbox.width) - floor(person_bbox.x + 0.25*person_bbox.width) + 1,
										   floor(person_bbox.y + 0.75*person_bbox.height) - floor(person_bbox.y + 0.25*person_bbox.height) + 1 )  );

				std::vector<float> depths;

				for (int i =0; i<depth_roi.rows; i++)
				{
					float *dptr = depth_roi.ptr<float>(i);
					for(int j = 0; j<depth_roi.cols; j++)
					{
						if(dptr[j] == dptr[j]) //To eliminate nans
						{
							depths.push_back(dptr[j]);	
						}
					}
				}

				std::vector<float>::iterator dbegin = depths.begin();
				std::vector<float>::iterator dend = depths.end();
				Point3d center3d;
				if (depths.size() > 0)
				{
					std::sort(dbegin, dend);
					double avg_d = depths[floor(depths.size() / 2.0)];
					// each_person.width3d = fabs((cam_model->left()).getDeltaX(each_person.bounding_box.width, avg_d));
					
					center3d = (cam_model_.left()).projectPixelTo3dRay(center2d);
					center3d = (avg_d / center3d.z) * center3d;
				}



				person_measurement.header.stamp = ros::Time::now();
				person_measurement.header.frame_id = image->header.frame_id;
				person_measurement.name = "Person_tracker";
				person_measurement.object_id = "Person1";
				person_measurement.pos.x = center3d.x;
				person_measurement.pos.y = center3d.y;
				person_measurement.pos.z = center3d.z;
				person_measurement.reliability = 1.0;

				people_msgs::PositionMeasurementArray pos_array;
				pos_array.header.stamp = ros::Time::now();
				pos_array.header.frame_id = image->header.frame_id;
				pos_array.people.push_back(person_measurement);
				people_tracker_measurements_pub.publish(pos_array);


				//calculate depth
				//publish location
			}
			else
			{
				is_tracking = false;
			}
			// ROS_INFO("Hi");
		}
		
	}

		// ROS_INFO("Total size is %d", tracker_list.size());
		



		// // ROS_INFO("HI");
		// cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");
		// cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image);
		// cv::Mat depth_32fc1 = cv_depth_ptr->image;
		// if (depth_image->encoding != "32FC1")
		// {
		// 	cv_depth_ptr->image.convertTo(depth_32fc1, CV_32FC1, 0.001);
		// }
		// cam_model_.fromCameraInfo(c1_info, c2_info);

		// Rect2d bbox(124, 32, 200, 280); 
		// Mat depth_roi(  depth_32fc1, Rect( floor(bbox.x + 0.25*bbox.width),
		// 							   floor(bbox.y + 0.25*bbox.height),
		// 							   floor(bbox.x + 0.75*bbox.width) - floor(bbox.x + 0.25*bbox.width) + 1,
		// 							   floor(bbox.y + 0.75*bbox.height) - floor(bbox.y + 0.25*bbox.height) + 1 )  );

		// std::vector<float> depths;
		// for (int i =0; i<depth_roi.rows; i++)
		// {
		// 	float *dptr = depth_roi.ptr<float>(i);
		// 	for(int j = 0; j<depth_roi.cols; j++)
		// 	{
		// 		if(dptr[j] == dptr[j]) //To eliminate nans
		// 		{
		// 			depths.push_back(dptr[j]);	
		// 		}
		// 	}
		// }
		
		// cv::Point2d center2d;
		// cv::Point3d center3d;
		// center2d = Point2d(bbox.x + (bbox.width/2.0),
		// 							 bbox.y + (bbox.height/2.0)  );

		// std::vector<float>::iterator dbegin = depths.begin();
		// std::vector<float>::iterator dend = depths.end();

		// if (depths.size() > 0)
		// {
		// 	std::sort(dbegin, dend);
		// 	double avg_d = depths[floor(depths.size() / 2.0)];
		// 	// each_person.width3d = fabs((cam_model->left()).getDeltaX(each_person.bounding_box.width, avg_d));
		// 	center3d = (cam_model_.left()).projectPixelTo3dRay(center2d);
		// 	center3d = (avg_d / center3d.z) * center3d;
		// }

		// ROS_INFO("centre3d x is %f", center3d.x);
		// ROS_INFO("centre3d y is %f", center3d.y);
		// ROS_INFO("centre3d z is %f", center3d.z);

		// float distance = get_shortest_distance(&center3d);
		// // ROS_INFO("distance");

		// // int id = get_new_ID();
		// // if(distance != -1.0)
		// // {
		// // 	if(distance)
		// // }
		// // else
		// if(distance > 0.2 || distance == -1.0)
		// {

		// 	// tracker_list[get_new_ID()] = center3d;
		// }
		
		// else
		// {
		// 	ROS_INFO("distance is %f", distance);
		// }


	

	float People_tracker_KCF::get_shortest_distance(Point3d *centre3d)
	{

		float min_value = FLT_MAX;
		
		for (auto i = tracked_points.begin(); i != tracked_points.end(); i++) 
		{ 	

			Point3d this_point = i->second;
			// ROS_INFO("existing %f", this_point.x);
			// ROS_INFO("existing %f", this_point.y);
			// ROS_INFO("existing %f", this_point.z);

			// ROS_INFO("new %f", centre3d->x);
			// ROS_INFO("new %f", centre3d->y);
			// ROS_INFO("new %f", centre3d->z);
			
			Point3d distance_vec = *centre3d - this_point; 

			float distance = sqrt(  (distance_vec.x*distance_vec.x)  + (distance_vec.y*distance_vec.y)   + (distance_vec.z*distance_vec.z)   );
			if(distance < min_value)
			{
				min_value = distance;
			}
		} 

		if(min_value == FLT_MAX)
		{
			return -1.0;
		}
		else
		{
			return min_value;
		}

		
		
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "people_tracking_kcf");

	People_Tracker::People_tracker_KCF People_tracker_KCF(ros::this_node::getName());

	ROS_INFO("Hi");

	return 0;
}