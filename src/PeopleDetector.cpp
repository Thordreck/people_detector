#include "PeopleDetector.h"

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/timer.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>
#include <iostream>


using namespace ros;
using namespace std;
using namespace boost;
using namespace message_filters;
using namespace sensor_msgs;
using namespace visualization_msgs;


PeopleDetector::PeopleDetector() : MIN_INTERSECT_(0.3)
{

	//Subscription and synchronization
	imgSub_.subscribe(nh_, "/bumblenode/right/image_raw", 20);
	cam_infoSub_.subscribe(nh_, "/bumblenode/right/camera_info", 20);
	laserfrontSub_.subscribe(nh_, "/scanfront", 50);
	laserbackSub_.subscribe(nh_, "/scanback", 50);

	sync_ = new Synchronizer<MySincPolicy>(MySincPolicy(10), imgSub_, cam_infoSub_, laserfrontSub_, laserbackSub_);
	sync_->registerCallback(boost::bind(&PeopleDetector::callback, this, _1, _2, _3, _4));	


	//Laser and image detectors
	imd_ = new ImageDetector(1,1,1);	
	ld_  = new LaserDetector();

	//Transform listener
	tf_listener_ = new tf::TransformListener();
	
	//People publisher
	peoplePub_ = nh_.advertise<MarkerArray>("people_marker", 1);

	return;
}

PeopleDetector::~PeopleDetector()
{
	delete imd_;
	delete ld_;
	delete sync_;
	delete tf_listener_;
	return;
}


void PeopleDetector::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg, const LaserScanConstPtr& laserfront_msg, const LaserScanConstPtr& laserback_msg)
{

	//Get current frame time
	Time time = laserfront_msg->header.stamp;

	timer t;

	vector<tf::Point> laser_legs;
	vector<cv::Rect> image_ROI, laser_ROI;

	//Extract image from ros message and convert it to openCV Mat format
	cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

	//Get camera info for 3D projection onto images
	cam_model_.fromCameraInfo(info_msg);

	//Run both image and laser based person detector
	getLaserDetection(laserfront_msg, laser_legs, laser_ROI, image);
	getImageDetection(image_ROI, image);

	for(int i=0; i<laser_ROI.size(); i++)
		cv::rectangle(image,laser_ROI[i], cv::Scalar(0,0,255), 2);	

	for(int i=0; i<image_ROI.size(); i++)
		cv::rectangle(image,image_ROI[i], cv::Scalar(255,0,0), 2);	


	//Merge laser and image data
	vector<mergedData> merged_data;

	mergeData(merged_data, image_ROI, laser_ROI, laser_legs);

	for(int i=0; i<merged_data.size(); i++)
		cv::rectangle(image,merged_data[i].ROI, cv::Scalar(0,255,0), 1);	

	//Update people data
	//updatePeople(merged_data, time);

	//Remove people who have not been updated for a while and their lifetime has reached zero
	//TODO: avoid using erase method
	for(int i=0; i<people_.size(); i++)
	{
		if(people_[i].getLifeTime() <= 0 )
			people_.erase(people_.begin() + i--);
	}

	//Publish detected people
	//publishPeople();
	
	//printPeople(image, time);

	cv::imshow("Frame", image);
	cv::waitKey(1);
	cout << "TIEMPO: " << t.elapsed() << endl;
	cout << "------------------------------" << endl;
}


void PeopleDetector::updatePeople(vector<mergedData>& merged_data, ros::Time time)
{
	//TODO: unify updatePeople and mergeData functions code in single general function, since the merge algorightm is the same

	int merge_index; //ROI laser index to merge with current image data
	double merge_area;  //Area of the candidate to merge

	for(int i=0; i < people_.size(); i++)
	{
		merge_index = -1;
		merge_area = 0.0;
		//Get person ROI: first we get the position of the person in global coordinates, 
		//transform them into base_link coordinates and get the ROI
		cv::Rect r1 = point3dToROI( globalCoordinatesToLocal( people_[i].getPos(), people_[i].getTime() ), people_[i].getTime() );

		for(int j=0; j < merged_data.size(); j++)
		{
			cv::Rect r2 = merged_data[j].ROI;
			double area = (r1 & r2).area();
			if( area > merge_area && area >= MIN_INTERSECT_ * min(r1.area(), r2.area()) )
			{
				merge_index = j;
				merge_area = area;
			}
		}
		//If a candidate has been found, the person is updated with the new info
		//and remove merged data used
		if(merge_index != -1)
		{
			mergedData data = merged_data[merge_index];

			if(data.detection != e_image)
			{
				bool withImage = (data.detection == e_laser_image); 
				people_[i].update(localCoordinatesToGlobal(data.pos, time), time, withImage);
			}
			else
			{
				try{
					tf::StampedTransform transform;
					tf_listener_->waitForTransform("/bumblebee", "/odom", time, ros::Duration(0.2));
					tf_listener_->lookupTransform("/bumblebee", "/odom", time, transform);

					int cx = data.ROI.x + (int) data.ROI.width/2;
					int cy = data.ROI.y + data.ROI.height;					
					
					people_[i].update(cx, cy, transform, time);
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s", ex.what());
					people_[i].update(time);
				}

			}
			merged_data.erase(merged_data.begin() + merge_index);
		}
		//If not, position is estimated using kalman filter 
		else
			people_[i].update(time);
	}
	//Merged data not used is added as new people
	for(int i=0; i < merged_data.size(); i++)
	{
		mergedData data = merged_data[i];
		//Ignore data obtained only from image detector. Image data is only used to update.
		if(data.detection != e_image)
			people_.push_back( Person( localCoordinatesToGlobal(data.pos, time), time) );
	}
	return;
}


//Returns the projected 3D world point in base_link frame in the unrectified image
cv::Point2d PeopleDetector::point3dTo2d(tf::Point point3d)
{
	//Correct transform to follow camera coordinates system standard
	//TODO: fix tf publisher and use camera info messages to get distance between stereo cameras
	cv::Point3d pt_cv  (-1*(point3d.y() + 0.06), -1*point3d.z(), point3d.x());	
	cv::Point2d point_2d_rect = cam_model_.project3dToPixel(pt_cv);

	//Points are unrectified since we are using raw images
	return cam_model_.unrectifyPoint(point_2d_rect);
}

//Returns ROI of person using 3D point on base_link frame
cv::Rect PeopleDetector::point3dToROI(tf::Point point3d, Time time)
{

	cv::Point2d uv_botright, uv_topleft;
	//Static transformations between laser, camera and base frames are obtained only once at the beginning 
	try
	{
		tf::StampedTransform transform;
        	tf_listener_->waitForTransform("/bumblebee",  "/base_link", time, ros::Duration(0.5));
        	tf_listener_->lookupTransform("/bumblebee",  "/base_link", time, transform);
		tf::Point bottom_pt = transform * point3d;

		uv_botright = point3dTo2d(bottom_pt + tf::Point(0, 0.25, 0));
		uv_topleft = point3dTo2d(bottom_pt + tf::Point(0,-0.25,1.80));
        }
    	catch (tf::TransformException ex)
	{
        	ROS_WARN("Cannot convert 3d point to ROI: %s", ex.what());
        }
	
	return cv::Rect(uv_topleft, uv_botright);

}

//Transform from robot relative coordinates to global coordinates using odometry info
tf::Point PeopleDetector::localCoordinatesToGlobal(tf::Point local_point, ros::Time time)
{
	tf::StampedTransform transform;
	try{
		tf_listener_->waitForTransform("/odom", "/base_link", time, ros::Duration(0.2));
		tf_listener_->lookupTransform("/odom", "/base_link", time, transform);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		return local_point;
	}
	return transform * local_point;
}

//Transform from robot relative coordinates to global coordinates using odometry info
tf::Point PeopleDetector::globalCoordinatesToLocal(tf::Point global_pt, ros::Time time)
{
	tf::StampedTransform transform;
	try{
		tf_listener_->waitForTransform("/base_link", "/odom", time, ros::Duration(0.2));
		tf_listener_->lookupTransform("/base_link", "/odom", time, transform);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		return global_pt;
	}
	return transform * global_pt;
}

//Merge laser and image data. The merge is undergone when laser and image ROI detection intersects with a percentage high enough. As image detection ROI are usually bigger than
//laser detection ROIs, they take precedence.
void PeopleDetector::mergeData(vector<mergedData>& v_merged_data, vector<cv::Rect>& image_ROI, vector<cv::Rect>& laser_ROI, vector<tf::Point>& laser_legs)
{
	//TODO: unify updatePeople and mergeData functions code in single general function, since the merge algorightm is the same
	int merge_index; //ROI laser index to merge with current image data
	double merge_area;  //Area of the candidate to merge

	for(int i=0; i < image_ROI.size(); i++)
	{
		merge_index = -1;
		merge_area = 0.0;
		cv::Rect r1 = image_ROI[i];

		for(int j=0; j < laser_ROI.size(); j++)
		{
			cv::Rect r2 = laser_ROI[j];
			double area = (r1 & r2).area();
			if( area > merge_area && area >= MIN_INTERSECT_ * min(r1.area(), r2.area()) )
			{
				merge_index = j;
				merge_area = area;
			}
		}
		mergedData merged_data;
		merged_data.ROI = image_ROI[i];

		//If a candidate has been found, we merge both laser and image data
		if(merge_index != -1)
		{
			merged_data.pos = laser_legs[merge_index];
			merged_data.detection = e_laser_image;
		}
		//If not, only image data is used
		else
			merged_data.detection = e_image;
	
		v_merged_data.push_back(merged_data);
		//Remove laser and image detection used in the merge (if any)
		//TODO: avoid using erase() method at all, as it is slow
		if(merge_index != -1)
		{
			laser_ROI.erase(laser_ROI.begin() + merge_index);
			laser_legs.erase(laser_legs.begin() + merge_index);
		}	
		image_ROI.erase(image_ROI.begin() + i--);
	}
	
	//Laser data not merged is added on its own
	mergedData merged_data;
	merged_data.detection = e_laser;

	for(int i=0; i < laser_ROI.size(); i++)
	{
		merged_data.ROI = laser_ROI[i];
		merged_data.pos = laser_legs[i];
		v_merged_data.push_back(merged_data);
	}
	return;	
}

void PeopleDetector::publishPeople()
{
	MarkerArray marker_array;
	Marker marker_init;

	marker_init.header.frame_id = "/base_link";
	marker_init.ns = "people_marker";
	marker_init.type = Marker::ARROW;
	marker_init.action = Marker::ADD;

	geometry_msgs::Point p;
	marker_init.points.push_back(p);
	
	marker_init.scale.x = 0.03;
	marker_init.scale.y = 0.05;
	marker_init.scale.z = 0.01;

	marker_init.color.r = 0.0f;
	marker_init.color.g = 1.0f;
	marker_init.color.b = 0.0f;
	marker_init.color.a = 1.0;
 
	for(int i=0; i<people_.size(); i++)
	{
		Marker marker = marker_init;
		marker.header.stamp = people_[i].getTime();	
		marker.id = people_[i].getId();
		tf::Point local_pt = globalCoordinatesToLocal( people_[i].getPos(), people_[i].getTime());
		p.x = local_pt.x();
		p.y = local_pt.y();
		p.z = 0;
		marker.points.push_back(p);
		marker_array.markers.push_back(marker);
	}	

	peoplePub_.publish(marker_array);
	return;
}

//Run laser based detector and filter results (aka ROI inside each other)
void PeopleDetector::getLaserDetection(const LaserScanConstPtr& laser_msg, vector<tf::Point>& laser_legs, vector<cv::Rect>& laser_ROI, cv::Mat image)
{
	vector<tf::Point> laser_legs_unfiltered, laser_legs_filtered;
	vector<cv::Rect> laser_ROI_unfiltered;

	//People's legs detection using laser data
	ld_->scan_message(laser_legs_unfiltered, laser_msg);

	for(int i=0; i< laser_legs_unfiltered.size(); i++)
	{
		cv::Rect ROI = point3dToROI(laser_legs_unfiltered[i], laser_msg->header.stamp);

		//First, check if whether the ROI is fully contained in the image as a prefilter
		cv::Rect rectsIntersection = ROI & cv::Rect(cv::Point(0,0),image.size());
		if( (ROI & rectsIntersection) == ROI )
		{
			laser_ROI_unfiltered.push_back(ROI);
			laser_legs_filtered.push_back(laser_legs_unfiltered[i]);
			//laser_ROI.push_back(ROI);
			//laser_legs.push_back(laser_legs_unfiltered[i]);
		}
	}
	//Remove ROIs that are fully contained inside bigger ROIs
	int i,j;
	for(i=0; i< laser_ROI_unfiltered.size(); i++)
	{
		cv::Rect r = laser_ROI_unfiltered[i];

		for(j = 0; j< laser_ROI_unfiltered.size(); j++)
		{
			if(j!=i && (r & laser_ROI_unfiltered[j]) == r)
				break;
		}
		if(j == laser_ROI_unfiltered.size())
		{
			laser_ROI.push_back(laser_ROI_unfiltered[i]);
			laser_legs.push_back(laser_legs_filtered[i]);
		}
	}
	return;	
}

void PeopleDetector::getImageDetection(vector<cv::Rect>& image_ROI, cv::Mat image)
{
	//Downscale image
	cv::Mat image_downscaled;
	cv::resize(image, image_downscaled,cv::Size(), 0.5,0.5);
	
	vector<cv::Rect> ROI_unfiltered;
	imd_->detectPeople(image_downscaled,ROI_unfiltered);

	//Upscale rects so it can be used on full sized image
	for(int i = 0; i < ROI_unfiltered.size(); i++)
	{
		ROI_unfiltered[i].x = 2*ROI_unfiltered[i].x;
		ROI_unfiltered[i].y = 2*ROI_unfiltered[i].y;
		ROI_unfiltered[i] += cv::Size(ROI_unfiltered[i].width, ROI_unfiltered[i].height);

		//TODO: use ROIto3dPoint instead		
		//Projection will be done using the middle base point of the rectangle
		cv::Point2d bot_mid(ROI_unfiltered[i].x + (int) ROI_unfiltered[i].width/2, ROI_unfiltered[i].y + ROI_unfiltered[i].height);	
		cv::Point2d bot_mid_rect = cam_model_.rectifyPoint(bot_mid);
	
		//Get the ray projection. The unit vector we get is in the camera coordinate system.
		cv::Point3d unit_vect_cam = cam_model_.projectPixelTo3dRay(bot_mid_rect);
		//Convert from camera coordinate system to tf system
		tf::Point unit_vect(unit_vect_cam.z, -1*unit_vect_cam.x, -1*unit_vect_cam.y);

		//Save only rectangles whose 3d ray projection goes through the floor (rectangles on upper positions cannot belong to real full people)
		if(unit_vect.z() < 0)
			image_ROI.push_back(ROI_unfiltered[i]);
	}
	return;
}

/*
void PeopleDetector::printPeople(cv::Mat image, ros::Time time)
{
	for(int i=0; i<people_.size(); i++)
	{
		tf::Point local_pt = globalCoordinatesToLocal(people_[i].getPos(), time);
		
		if(local_pt.x() < 0)
			continue;
		cv::Point2d uv_botright = point3dTo2d(local_pt + tf::Point(0,-0.5, -0.33-0.93));
		cv::Point2d uv_topleft = point3dTo2d(local_pt+ tf::Point(0,0.5,-0.33-0.93+1.90));
		cv::Rect ROI(uv_topleft,uv_botright);

		char text[100];
		sprintf(text,"ID: %d | ERROR_COV: %f | FIAB: %d", people_[i].getId(), people_[i].getErrorCov(), people_[i].getFiab());
		cv::rectangle(image,ROI, cv::Scalar(0,255,0));	
		cv::putText(image, text, uv_topleft, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,255,0));
	}
	return;
}
*/
