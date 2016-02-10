#ifndef CORRECT_DATA_H
#define CORRECT_DATA_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class CorrectData{
	
		ros::NodeHandle	nh_;
		image_transport::ImageTransport		it_;
		image_transport::CameraPublisher	camera_pub_;
		image_transport::CameraSubscriber	camera_sub_;
		tf::TransformBroadcaster		tf_broadcaster_;
		tf::TransformListener			tf_listener_;
	
	public:
		CorrectData();
		void videoCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);


};

#endif
