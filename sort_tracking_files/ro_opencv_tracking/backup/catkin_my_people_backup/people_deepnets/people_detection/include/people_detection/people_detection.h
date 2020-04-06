#ifndef PEOPLE_DETECTION_H
#define PEOPLE_DETECTION_H

#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <boost/thread/mutex.hpp>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "image_geometry/stereo_camera_model.h"
#include "cv_bridge/cv_bridge.h"
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <object_detection_msgs/BoundingBox.h>
#include <object_detection_msgs/BoundingBoxMeasurementsArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>

#include <opencv/cv.hpp>
#include "image_geometry/stereo_camera_model.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>

#define NUM_CLASSES 21

using namespace std;
using namespace cv;
using namespace dnn;

namespace People
{

	struct Detections
	{
		cv::Point2d center2d;
		cv::Point3d center3d;
		double width2d;
		double height2d;
		cv::Rect bounding_box;
		double width3d;
		// double width2d;
	};

	class PeopleDetector
	{
		public:
			ros::NodeHandle nh_;
			//boost::mutex connect_mutex_;
			string image_topic_name;
			string depth_topic_name;
			string camera_name;
			string camera_info;
			string image_topic;
			string depth_topic;
			string camera_rgb_info_topic;
			string camera_depth_info_topic;
			string rgb_ns;
			string depth_ns;
			image_transport::ImageTransport it_;
			image_transport::SubscriberFilter image_sub_;
			image_transport::SubscriberFilter depth_image_sub_;
			message_filters::Subscriber<sensor_msgs::CameraInfo> c1_info_sub_; /**< rgb camera info msg. */
  			message_filters::Subscriber<sensor_msgs::CameraInfo> c2_info_sub_; /**<depth camera info msg. */
  			// image_transport::Publisher test_pub = it_.advertise("test_camera/image", 1);
  			// image_transport::Publisher test_depth_pub = it_.advertise("test_camera/depth_image", 1);
  			image_geometry::StereoCameraModel cam_model_; 
  			ros::Publisher pos_array_pub_;
  			ros::Publisher bbox_pub;
			typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactDepthPolicy; /**< Sync policy for exact time with depth. */
			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateDepthPolicy; /**< Sync policy for approx time with depth. */
			typedef message_filters::Synchronizer<ExactDepthPolicy> ExactDepthSync;
			typedef message_filters::Synchronizer<ApproximateDepthPolicy> ApproximateDepthSync;
			boost::shared_ptr<ExactDepthSync> exact_depth_sync_;
			boost::shared_ptr<ApproximateDepthSync> approximate_depth_sync_;
			
			//Parameters to refer to the model location. Needs to be changed as a ros parameter
			String model_txt;
			String model_bin;
			
			Net net;

			/**To set these paramters as private**/
			bool approx = true;
			int  queue_size = 1000;

			PeopleDetector(std::string name, String model_txt, String model_bin);

			void connect_callback();

			vector<Detections> detect_people(const cv::Mat image, double confidence_threshold, const cv::Mat depth_image, int object_id, image_geometry::StereoCameraModel *cam_model);

			void image_and_depth_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info);

			void publish_people(vector<Detections> people_detections, std_msgs::Header header);
	};

};



#endif