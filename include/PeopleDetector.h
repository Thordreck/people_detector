#ifndef PEOPLE_DETECTOR_H
#define PEOPLE_DETECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv/cv.h>

#include "ImageDetector.h"
#include "LaserDetector.h"
#include "Person.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan> MySincPolicy;
	
class PeopleDetector
{
		ros::NodeHandle							nh_;

		ImageDetector 							*imd_;
		LaserDetector 							*ld_;

		image_geometry::PinholeCameraModel				cam_model_;

		tf::TransformListener						*tf_listener_;
		tf::StampedTransform						tf_camera_base_link_;
		tf::StampedTransform						tf_laser_base_link_;

		message_filters::Subscriber<sensor_msgs::Image>			imgSub_;
		message_filters::Subscriber<sensor_msgs::CameraInfo>		cam_infoSub_;
		message_filters::Subscriber<sensor_msgs::LaserScan>		laserSub_;
		message_filters::Synchronizer<MySincPolicy>			*sync_;

		ros::Publisher							peoplePub_;

		std::vector<Person>						people_;
		const double							max_step_dist_; //Average person step size (50cm)
		const double							max_img_dist_; //Average person step size (50cm)
		const int							max_inactive_time_;
		const double							sensitivity_;
		const double							max_error_cov_;

		cv::Point2d		point3dTo2d(tf::Point point3d);
		tf::Point		ROIto3dPoint(cv::Rect ROI);
		void			updatePeople(std::vector<Person>& people_aux,std::vector<tf::Point>& , ros::Time time);
		int			getClosestPoint(tf::Point point, std::vector<Person>& candidates,double max_dist);
		int			getClosestPoint(cv::Point2d laser_pt, std::vector<cv::Point2d>& image_pts,double max_dist);
		void			discardInactives();
		tf::Point		localCoordinatesToGlobal(tf::Point leg_point, ros::Time time);
		void			mergeLaserAndImageData( std::vector<cv::Rect>& image_ROI, cv::Mat image, ros::Time time);
		tf::Point		globalCoordinatesToLocal(tf::Point global_pt, ros::Time time);
		void			publishPeople();
		void			getLaserDetectionROI(std::vector<tf::Point>& laser_legs, std::vector<cv::Rect>& laser_ROI, cv::Mat image);
		void			getImageDetectionROI(std::vector<cv::Rect>& image_ROI,cv::Mat image);
		void			printPeople(cv::Mat image, ros::Time time);
		
	public:
			PeopleDetector(char *imageTopic, char* infoTopic ,char*laserTopic);
			~PeopleDetector();
		void 	callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, const sensor_msgs::LaserScan::ConstPtr& laser_msg);			
};

#endif
