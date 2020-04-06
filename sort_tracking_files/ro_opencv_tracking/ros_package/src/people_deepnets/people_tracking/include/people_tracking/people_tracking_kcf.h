#ifndef PEOPLE_TRACKING_KCF
#define PEOPLE_TRACKING_KCF


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <object_detection_msgs/BoundingBox.h>
#include <object_detection_msgs/BoundingBoxMeasurementsArray.h>
#include "image_geometry/stereo_camera_model.h"
#include <image_transport/subscriber_filter.h>

#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>
#include<opencv2/tracking.hpp>



using namespace std;
using namespace cv;

namespace People_Tracker
{
	class People_tracker_KCF
	{
		public:
			ros::NodeHandle nh_;
			string image_topic_name;
			string depth_topic_name;
			string camera_name;
			string camera_info;
			string camera_rgb_info_topic;
			string camera_depth_info_topic;
			string rgb_ns;
			string depth_ns;
			string image_topic;
			string depth_topic;
			string bbox_topic_name;
			string bbox_ns;
			string bbox_topic;
			image_geometry::StereoCameraModel cam_model_; 
			// vector< Ptr<TrackerKCF> > tracker_list;

			static unsigned int id_seed;

			bool is_tracking;

			unordered_map<unsigned int, Point3d> tracked_points; 
			unordered_map<unsigned int, Ptr<TrackerKCF>> tracker_list; 

			 Ptr<TrackerKCF> person_tracker;
			 Rect2d person_bbox; 

			 people_msgs::PositionMeasurement person_measurement;

			

			image_transport::ImageTransport it_;
			image_transport::SubscriberFilter image_sub_;
			image_transport::SubscriberFilter depth_image_sub_;
			message_filters::Subscriber<object_detection_msgs::BoundingBoxMeasurementsArray> bbox_sub;
			message_filters::Subscriber<sensor_msgs::CameraInfo> c1_info_sub_;
			message_filters::Subscriber<sensor_msgs::CameraInfo> c2_info_sub_;
			ros::Publisher people_tracker_measurements_pub;
			int  queue_size = 600;
			
			// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, object_detection_msgs::BoundingBoxMeasurementsArray> ApproximateBoundingBoxPolicy;
			// typedef message_filters::Synchronizer<ApproximateBoundingBoxPolicy> ApproximateBoundingBoxSync;
			// boost::shared_ptr<ApproximateBoundingBoxSync> approximate_bbox_sync;

			// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximateBoundingBoxPolicyTest;
			// typedef message_filters::Synchronizer<ApproximateBoundingBoxPolicyTest> ApproximateBoundingBoxSyncTest;
			// boost::shared_ptr<ApproximateBoundingBoxSyncTest> approximate_bbox_sync_test;

			// boost::shared_ptr<ApproximateBoundingBoxSyncTest> approximate_bbox_sync_test_2;
			// bool done = false;

			// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateDepthPolicy; *< Sync policy for approx time with depth. 
			// typedef message_filters::Synchronizer<ApproximateDepthPolicy> ApproximateDepthSync;
			// boost::shared_ptr<ApproximateDepthSync> approximate_depth_sync_;
			
			
			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, object_detection_msgs::BoundingBoxMeasurementsArray> ApproximateBoundingBoxPolicy; /**< Sync policy for approx time with depth. */
			typedef message_filters::Synchronizer<ApproximateBoundingBoxPolicy> ApproximateBoundingBoxSync;
			boost::shared_ptr<ApproximateBoundingBoxSync> approximate_bbox_sync;
			

			People_tracker_KCF(string name);
			void connect_callback();
			// void image_and_bbox_cb(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::CameraInfo::ConstPtr &c1_info, const object_detection_msgs::BoundingBoxMeasurementsArray::ConstPtr &bbox_measurements_array );
			// void image_and_bbox_cb_test(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::CameraInfo::ConstPtr &c1_info);

			// void image_and_bbox_cb_test_2(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::CameraInfo::ConstPtr &c1_info);
			// void publish_people(vector<Detections> people_detections, std_msgs::Header header);
			
			// void track_people(const cv::Mat image);
			// void image_and_depth_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info);
			void image_depth_bbox_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info, const object_detection_msgs::BoundingBoxMeasurementsArray::ConstPtr &bbox_measurements_array );

			

			float get_shortest_distance(Point3d* centre3d);

			static  unsigned int get_new_ID()
			{
				id_seed++;
				return id_seed;
			} 
	};
};

#endif