// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <sstream>
// #include <boost/thread/mutex.hpp>
// #include <ros/console.h>
// #include <boost/filesystem.hpp>
// #include <image_transport/subscriber_filter.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include "image_geometry/stereo_camera_model.h"
// #include "cv_bridge/cv_bridge.h"
// #include <people_msgs/PositionMeasurement.h>
// #include <people_msgs/PositionMeasurementArray.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/dnn.hpp>
// #include <opencv2/dnn.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/core/utils/trace.hpp>

#include "people_detection/people_detection.h"

namespace People
{
	// class PeopleDetector
	// {
		// public:
			
			// ros::NodeHandle nh_;
			// //boost::mutex connect_mutex_;
			// string image_topic_name;
			// string depth_topic_name;
			// string camera_name;
			// string camera_info;
			// string image_topic;
			// string depth_topic;
			// string camera_rgb_info_topic;
			// string camera_depth_info_topic;
			// string rgb_ns;
			// string depth_ns;
			// image_transport::ImageTransport it_;
			// image_transport::SubscriberFilter image_sub_;
			// image_transport::SubscriberFilter depth_image_sub_;
			// message_filters::Subscriber<sensor_msgs::CameraInfo> c1_info_sub_; /**< rgb camera info msg. */
  	// 		message_filters::Subscriber<sensor_msgs::CameraInfo> c2_info_sub_; /**<depth camera info msg. */
  	// 		image_transport::Publisher test_pub = it_.advertise("test_camera/image", 1);
  	// 		image_geometry::StereoCameraModel cam_model_; 
  	// 		ros::Publisher pos_array_pub_;
			// typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactDepthPolicy; /**< Sync policy for exact time with depth. */
			// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateDepthPolicy; /**< Sync policy for approx time with depth. */
			// typedef message_filters::Synchronizer<ExactDepthPolicy> ExactDepthSync;
			// typedef message_filters::Synchronizer<ApproximateDepthPolicy> ApproximateDepthSync;
			// boost::shared_ptr<ExactDepthSync> exact_depth_sync_;
			// boost::shared_ptr<ApproximateDepthSync> approximate_depth_sync_;
			
			// //Parameters to refer to the model location. Needs to be changed as a ros parameter
			// String model_txt;
			// String model_bin;
			
			// Net net;


			// /**To set these paramters as private**/
			// bool approx = false;
			// bool  queue_size = 5;
			/************************************/

			PeopleDetector::PeopleDetector(std::string name, String model_txt, String model_bin):it_(nh_),model_txt(model_txt),model_bin(model_bin)
			{
				ROS_INFO("Creating people detector node");
				camera_name = nh_.resolveName("camera");
				image_topic_name = nh_.resolveName("image_topic");
				rgb_ns = nh_.resolveName("rgb_ns");
				depth_ns = nh_.resolveName("depth_ns");
				depth_topic_name = nh_.resolveName("depth_topic");
				camera_info = nh_.resolveName("camera_info");

				image_topic = ros::names::clean(camera_name + "/" + rgb_ns + "/" + image_topic_name);
				depth_topic = ros::names::clean(camera_name + "/" + depth_ns + "/" + depth_topic_name);
				
				camera_rgb_info_topic = ros::names::clean(camera_name + "/" + rgb_ns + "/" + camera_info);
				camera_depth_info_topic = ros::names::clean(camera_name + "/" + depth_ns + "/" + camera_info);

				// ROS_INFO("camera topic is %s: ", camera_name.c_str());
				// ROS_INFO("image topic is %s: ", image_topic.c_str());
				// ROS_INFO("rgb_ns topic is %s: ", rgb_ns.c_str());
				// ROS_INFO("depth_ns topic is %s: ", depth_ns.c_str());
				// ROS_INFO("depth topic topic is %s: ", depth_topic.c_str());
				// ROS_INFO("camera_rgb_info_topic topic is %s: ", camera_rgb_info_topic.c_str());
				// ROS_INFO("camera_depth_info_topic topic is %s: ", camera_depth_info_topic.c_str());
				// String model_txt = "/home/shreyas/object_detection_YOLO/ssd/realtime_object_recognition-master/MobileNetSSD_deploy.prototxt";
				// String model_bin = "/home/shreyas/object_detection_YOLO/ssd/realtime_object_recognition-master/MobileNetSSD_deploy.caffemodel";

				if(approx)
				{
					approximate_depth_sync_.reset(new ApproximateDepthSync(ApproximateDepthPolicy(queue_size), image_sub_, depth_image_sub_,c1_info_sub_, c2_info_sub_));
					approximate_depth_sync_->registerCallback(boost::bind(&PeopleDetector::image_and_depth_callback, this, _1, _2, _3, _4));
				}
				else
				{
					exact_depth_sync_.reset(new ExactDepthSync(ExactDepthPolicy(queue_size), image_sub_, depth_image_sub_,c1_info_sub_, c2_info_sub_));
					exact_depth_sync_->registerCallback(boost::bind(&PeopleDetector::image_and_depth_callback, this, _1, _2, _3, _4));
				}
			

				ros::SubscriberStatusCallback position_publisher_connect_callback = boost::bind(&PeopleDetector::connect_callback, this);
				
				ROS_INFO("Please subscribe to people_measurements.");

				{
					//boost::mutex::scoped_lock lock(connect_mutex_);
					bbox_pub = nh_.advertise<object_detection_msgs::BoundingBoxMeasurementsArray>("people_detector/bbox_measurements_array", 1, position_publisher_connect_callback, position_publisher_connect_callback);
				}

				net = readNetFromCaffe(model_txt, model_bin);

				// ros::MultiThreadedSpinner s(2);
    			ros::spin();
			}

			void PeopleDetector::connect_callback()
			{
				//boost::mutex::scoped_lock lock(connect_mutex_);
				if(bbox_pub.getNumSubscribers() == 0)
				{
					ROS_INFO("Please subscribe to people detectors outbound topics");
					image_sub_.unsubscribe();
					depth_image_sub_.unsubscribe();
					c1_info_sub_.unsubscribe();
					c2_info_sub_.unsubscribe();
				}
				else
				{
					image_sub_.subscribe(it_, image_topic, 3);
					depth_image_sub_.subscribe(it_, depth_topic, 3);
					c1_info_sub_.subscribe(nh_, camera_rgb_info_topic, 3);
					c2_info_sub_.subscribe(nh_, camera_depth_info_topic, 3);
				}
			}

			vector<Detections> PeopleDetector::detect_people(const cv::Mat image, double confidence_threshold, const cv::Mat depth_image, int object_id, image_geometry::StereoCameraModel *cam_model)
			{	
				vector<Detections> people_detections;
				Mat resize_image;
				Mat image_copy = image.clone();
				// ROS_INFO("Width is %d", image_copy.cols);
				// ROS_INFO("Height is %d", image_copy.rows);


				resize(image_copy, resize_image,  Size(300,300));
				
				Mat inputBlob = blobFromImage(resize_image, 0.007843, Size(300,300), Scalar(127.5, 127.5, 127.5), false);
				net.setInput(inputBlob, "data");
				Mat detection = net.forward("detection_out");
				Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

				//array CLASSES<string,21> = {"background", "aeroplane", "bicycle", "bird", "boat",
				//		"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
				//		"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
				//		"sofa", "train", "tvmonitor"};

				float confidenceThreshold = 0.2;
				// ROS_INFO("Hi");
				
				for (int i = 0; i < detectionMat.rows; i++)
				{
					float confidence = detectionMat.at<float>(i, 2);

					//ROS_INFO("Confidence is %f", confidence);

					if (confidence > confidenceThreshold)
					{
						int idx = static_cast<int>(detectionMat.at<float>(i, 1));
						if(idx == 15)
						{
							Detections each_person;
							int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * image.cols);
							int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * image.rows);
							int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * image.cols);
							int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * image.rows);

							// each_person.bounding_box = new Rect((int)xLeftBottom, (int)yLeftBottom,(int)(xRightTop - xLeftBottom),(int)(yRightTop - yLeftBottom));

							// Rect object((int)xLeftBottom, (int)yLeftBottom,
							// (int)(xRightTop - xLeftBottom),
							// (int)(yRightTop - yLeftBottom));
							// rectangle(image_copy, object, Scalar(0, 255, 0), 2);


							each_person.bounding_box = Rect((int)xLeftBottom, (int)yLeftBottom,
							(int)(xRightTop - xLeftBottom),
							(int)(yRightTop - yLeftBottom));

							each_person.center2d = Point2d(each_person.bounding_box.x + (each_person.bounding_box.width/2.0),
																 each_person.bounding_box.y + (each_person.bounding_box.height/2.0)  );
							each_person.width2d = each_person.bounding_box.width;
							each_person.height2d = each_person.bounding_box.height;

							//Following a pattern similar to ros people package. Getting median depth

							Mat depth_roi(  depth_image, Rect( floor(each_person.bounding_box.x + 0.25*each_person.bounding_box.width),
															   floor(each_person.bounding_box.y + 0.25*each_person.bounding_box.height),
															   floor(each_person.bounding_box.x + 0.75*each_person.bounding_box.width) - floor(each_person.bounding_box.x + 0.25*each_person.bounding_box.width) + 1,
															   floor(each_person.bounding_box.y + 0.75*each_person.bounding_box.height) - floor(each_person.bounding_box.y + 0.25*each_person.bounding_box.height) + 1 )  );

							// ROS_INFO("depth_rows is %d", depth_roi.rows);
							// ROS_INFO("depth_cols is %d", depth_roi.cols);
							// imshow("image",depth_roi);
							// waitKey(20);

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
							if (depths.size() > 0)
							{
								std::sort(dbegin, dend);
								double avg_d = depths[floor(depths.size() / 2.0)];
								each_person.width3d = fabs((cam_model->left()).getDeltaX(each_person.bounding_box.width, avg_d));
								
								each_person.center3d = (cam_model->left()).projectPixelTo3dRay(each_person.center2d);
								each_person.center3d = (avg_d / each_person.center3d.z) * each_person.center3d;
							}

							people_detections.push_back(each_person);   
	                        // ROS_INFO("Depth image x is %f", each_person.center3d.x);
	                        // ROS_INFO("Depth image y is %f", each_person.center3d.y);
	                        // ROS_INFO("Depth image z is %f", each_person.center3d.z);
	                        // ROS_INFO("____________________________________________");
	                        // ROS_INFO("Width 3d is %f", each_person.width3d);

	                        // rectangle(image_copy, object, Scalar(0, 255, 0), 2);
                    	}


    				}
    			}
    			
    			return people_detections;
			}

			void PeopleDetector::publish_people(vector<Detections> people_detections, std_msgs::Header header)
			{
				// ROS_INFO("Hi");
				object_detection_msgs::BoundingBoxMeasurementsArray bounding_box_array;
				bounding_box_array.header.stamp = ros::Time::now();
				bounding_box_array.header.frame_id = header.frame_id;
				int count = 0;

				for (Detections each_person: people_detections)
				{
					string name = "person" + to_string(count);
					// ROS_INFO("size is %f", each_person.height2d);
					object_detection_msgs::BoundingBox bbox_msg;
					bbox_msg.class_name  = name;
					bbox_msg.header.stamp = header.stamp;
					bbox_msg.header.frame_id = header.frame_id;
					bbox_msg.center.x = each_person.center3d.x;
					bbox_msg.center.y = each_person.center3d.y;
					bbox_msg.center.z = each_person.center3d.z;
					bbox_msg.bbox_coors.x = each_person.bounding_box.x;
					bbox_msg.bbox_coors.y = each_person.bounding_box.y;
					bbox_msg.bbox_coors.z = 0;
					bbox_msg.bbox_width   = each_person.width2d;
					bbox_msg.bbox_height   = each_person.height2d;
					bounding_box_array.bounding_box_array.push_back(bbox_msg);
					count += 1;
				}
				bbox_pub.publish(bounding_box_array);
			}

			void PeopleDetector::image_and_depth_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info)
			{
				// ROS_INFO("Number of subscribers is %d", pos_array_pub_.getNumSubscribers() );
				// ROS_INFO("Hi");
				// test_pub.publish(image);
				
				// test_depth_pub.publish(depth_image);

				cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");
				cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image);
				cv::Mat depth_32fc1 = cv_depth_ptr->image;
				if (depth_image->encoding != "32FC1")
				{
						cv_depth_ptr->image.convertTo(depth_32fc1, CV_32FC1, 0.001);
				}
				cam_model_.fromCameraInfo(c1_info, c2_info);

				vector<Detections> people_detections = detect_people(cv_image_ptr->image, 0.2, depth_32fc1, 1, &cam_model_);

				publish_people(people_detections, image->header);

				// ROS_INFO("people size is %d", people_detections.size());

    			// detector->detect_object(depth_32fc1, 0.2, depth_32fc1,1);

    			// vector<Box2D3D> people_vector = network_.detect_people_depth(cv_image_ptr->image, depth_32fc1, &cam_model_ );
			}
	//};
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "people_detector");
	String modelTxt = "/home/shreyas/object_detection_YOLO/ssd/realtime_object_recognition-master/MobileNetSSD_deploy.prototxt";
    String modelBin = "/home/shreyas/object_detection_YOLO/ssd/realtime_object_recognition-master/MobileNetSSD_deploy.caffemodel";
	People::PeopleDetector people_detector(ros::this_node::getName(), modelTxt, modelBin);
	// ROS_INFO("starting people detection");

	// // ros::NodeHandle n;
	// // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	// // ROS_INFO("Publishing node");
	// // ros::Rate loop_rate(10);
	// // std_msgs::String msg;
	// // std::stringstream ss;
	// // ss << "hello world ";
	// // msg.data = ss.str();
	// // ROS_INFO("%s", msg.data.c_str());
	// // chatter_pub.publish(msg);
	// // ros::spinOnce();
	// // loop_rate.sleep();
	return 0;




	// string CLASSES[] = {"background", "aeroplane", "bicycle", "bird", "boat",
 //        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
 //        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
 //        "sofa", "train", "tvmonitor"};

	// using namespace cv;
	// using namespace dnn;
	
	
	// //ROS_INFO("CV version is %s", cv::CV_VERSION);
	// // ROS_INFO("CV_MAJOR_VERSION is %d", cv::CV_MAJOR_VERSION);

	// //cout <<  cv::CV_MAJOR_VERSION;

	// String modelTxt = "/home/shreyas/object_detection_YOLO/ssd/realtime_object_recognition-master/MobileNetSSD_deploy.prototxt";
 //    String modelBin = "/home/shreyas/object_detection_YOLO/ssd/realtime_object_recognition-master/MobileNetSSD_deploy.caffemodel";
 //    Net net = readNetFromCaffe(modelTxt, modelBin);
	// String imageFile = "/home/shreyas/Desktop/persons/person_001.jpg";
	// Mat img = imread(imageFile);
	// Mat img2;
	// ROS_INFO("Image height is %d", img.size().height);
	// ROS_INFO("Image width is %d", img.size().width);
 //    resize(img, img2, Size(300,300));
 //    Mat inputBlob = blobFromImage(img2, 0.007843, Size(300,300), Scalar(127.5, 127.5, 127.5), false);
 //    // Mat inputBlob = blobFromImage(img2, 1.0, Size(300,300), Scalar(104.0, 177.0, 123.0), false);

 //    net.setInput(inputBlob, "data");
 //    Mat detection = net.forward("detection_out");
 //    Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
 //    ostringstream ss;

 //    float confidenceThreshold = 0.2;
 //    for (int i = 0; i < detectionMat.rows; i++)
 //    {
 //        float confidence = detectionMat.at<float>(i, 2);
 //        // cout << "confidence is : " << confidence << endl;

 //        if (confidence > confidenceThreshold)
 //        {
 //            cout << "confidence is : " << confidence << endl;
 //            int idx = static_cast<int>(detectionMat.at<float>(i, 1));

 //            int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * img.cols);
 //            int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * img.rows);
 //            int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * img.cols);
 //            int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * img.rows);

 //            ROS_INFO("Coordinates are %d, %d and %d, %d ", xLeftBottom,yLeftBottom,xRightTop,yRightTop);

 //            Rect object((int)xLeftBottom, (int)yLeftBottom,
 //                        (int)(xRightTop - xLeftBottom),
 //                        (int)(yRightTop - yLeftBottom));

 //            rectangle(img, object, Scalar(0, 255, 0), 2);

 //            cout << CLASSES[idx] << ": " << confidence << endl;

 //            ss.str("");
 //            ss << confidence;
 //            String conf(ss.str());
 //            String label = CLASSES[idx] + ": " + conf;
 //            int baseLine = 0;
 //            Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
 //            putText(img, label, Point(xLeftBottom, yLeftBottom),
 //                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
 //        }
 //    }
 //    namedWindow("test");
 //    moveWindow("test", 200,20);
 //    imshow("detections", img);
 //    waitKey();


 //    return 0;

}