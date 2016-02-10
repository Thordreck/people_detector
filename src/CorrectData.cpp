#include "CorrectData.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

CorrectData::CorrectData() : it_(nh_)
{
	camera_sub_ = it_.subscribeCamera("bumblenode/right/image_raw",5,&CorrectData::videoCallback,this);
	camera_pub_ = it_.advertiseCamera("bumblenode/right/image_fixed",5);	
}

void CorrectData::videoCallback(const sensor_msgs::ImageConstPtr& image_msg,const  sensor_msgs::CameraInfoConstPtr& info_msg)
{


	ros::Time now = ros::Time::now();
	tf::StampedTransform stamped_transform;
	tf::Transform transform;

	try
	{
		tf_listener_.waitForTransform("base_link","bumblebee",now, ros::Duration(2.0));
		tf_listener_.lookupTransform("base_link","bumblebee",now, stamped_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}


	transform.setOrigin(tf::Vector3(0.0,0.06,0.0) + stamped_transform.getOrigin());
	tf::Quaternion quaternion(0.5,0.5,0.5,0.5);
	transform.setRotation(quaternion);
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform,now,"base_link","bumblebee/right"));	

	//sensor_msgs::Image::Ptr image_msg_fixed = boost::make_shared<sensor_msgs::Image>();
	sensor_msgs::Image image_msg_fixed = *image_msg;
	image_msg_fixed.header.frame_id = "bumblebee/right";

	//sensor_msgs::CameraInfo::Ptr info_msg_fixed = boost::make_shared<sensor_msgs::CameraInfo>();
	sensor_msgs::CameraInfo info_msg_fixed = *info_msg;
	info_msg_fixed.header.frame_id = "bumblebee/right";	
	camera_pub_.publish(image_msg_fixed,info_msg_fixed,now);

	return;
}

