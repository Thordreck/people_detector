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


PeopleDetector::PeopleDetector(char *imageTopic, char* infoTopic, char* laserTopic) : max_step_dist_(0.7), max_img_dist_(1.0),max_inactive_time_(2), sensitivity_(0.5), max_error_cov_(1.0)
{
	
	//Subscription and synchronization
	imgSub_.subscribe(nh_,imageTopic,20);
	cam_infoSub_.subscribe(nh_,infoTopic,20);
	laserSub_.subscribe(nh_,laserTopic,50);

	sync_ = new Synchronizer<MySincPolicy>(MySincPolicy(10), imgSub_, cam_infoSub_, laserSub_);
	sync_->registerCallback(boost::bind(&PeopleDetector::callback, this, _1, _2, _3));	


	//Laser and image detectors
	imd_ = new ImageDetector(1,1,1);	
	ld_  = new LaserDetector("/home/viki/tfg_ws/src/people_detector/src/legs_detector/training_data/hypo1.dat");

	//Transform listener
	tf_listener_ = new tf::TransformListener();

	//Static transformations between laser, camera and base frames are obtained only once at the beginning 
	try
	{
        	tf_listener_->waitForTransform("/base_link",  "/bumblebee", ros::Time(0), ros::Duration(5));
        	tf_listener_->lookupTransform("/base_link",  "/bumblebee", ros::Time(0), tf_camera_base_link_);
        	tf_listener_->waitForTransform("/base_link",  "/laserfront", ros::Time(0), ros::Duration(5));
        	tf_listener_->lookupTransform("/base_link",  "/laserfront", ros::Time(0), tf_laser_base_link_);
        }
    	catch (tf::TransformException ex)
	{
        ROS_WARN("Static transform unavailable %s", ex.what());
        }
	
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


void PeopleDetector::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& info_msg, const LaserScanConstPtr& laser_msg)
{

	//Get current frame time
	Time time = laser_msg->header.stamp;

	timer t;

	vector<tf::Point> laser_legs;
	vector<cv::Rect> image_ROI, laser_ROI;

//	vector<Person> people_aux = people_;

//	people_.clear();

	//Extract image from ros message and convert it to openCV Mat format
	cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

	//Get camera info for 3D projection onto images
	cam_model_.fromCameraInfo(info_msg);

	//People's legs detection using laser data
	ld_->scan_message(laser_legs,laser_msg);

	getLaserDetectionROI(laser_legs,laser_ROI, image);
	for(int i=0; i<laser_ROI.size(); i++)
		cv::rectangle(image,laser_ROI[i], cv::Scalar(0,255,0));	

	//Update people data
	//updatePeople(people_aux, laser_legs, time);

	//Scan the whole downscaled image
	//getImageDetectionROI(image_ROI, image);

	//Merge laser and image data
	//mergeLaserAndImageData(image_ROI, image, time);

	//People reamining in people_aux vector have not been associated. We predict their position and delete them if necessary
	//discardInactives();	

	//Publish detected people
	//publishPeople();
	
	//printPeople(image, time);

	cv::imshow("Frame", image);
	cv::waitKey(1);
	cout << "TIEMPO: " << t.elapsed() << endl;
	cout << "------------------------------" << endl;
}

void PeopleDetector::updatePeople(vector<Person>& people_aux, vector<tf::Point>& laser_legs, ros::Time time)
{
	vector<int> index;
	int i,j;
	
	//Calculate closest people position to merged data
	for(i=0; i<laser_legs.size(); i++)
		index.push_back(getClosestPoint(laser_legs[i], people_aux, max_step_dist_) );
	
	//Check first those legs that share their associated person with no other 	
	for(i=0; i<index.size() && index[i] != -1; i++)
	{
		for(j=0; j<index.size(); j++)
			if(i!=j && index[i] == index[j])
				break;
		if(j == index.size())
		{
			people_aux[index[i]].correct(laser_legs[i], time);
			people_.push_back(people_aux[index[i]]);
			people_aux.erase(people_aux.begin() + index[i]);
			laser_legs.erase(laser_legs.begin() + i);
			index.erase(index.begin() + i--);
		}	
	}

	//Check the first pair of legs that share the same person
	bool pair_found = false;
	int closest_index;
	for(i=0; i<index.size() && index[i] != -1 && !pair_found; i++)
	{
		for(j=i+1; j<index.size() && !pair_found; j++)
		{
			if(index[i] == index[j])
			{
				closest_index = ( laser_legs[i].distance(people_aux[index[i]].getPos()) < laser_legs[j].distance(people_aux[index[i]].getPos())) ? i : j;
				people_aux[index[i]].correct(laser_legs[closest_index], time);
				people_.push_back(people_aux[index[i]]);
				people_aux.erase(people_aux.begin()+index[i]);
				laser_legs.erase(laser_legs.begin()+closest_index);
				pair_found = true;
			}
		}
	}	
	//If we found a pair, we have to recalculate again the distance of the remaining legs
	if(pair_found)
		updatePeople(people_aux, laser_legs, time);

	//Remaining merged data is treated as new detected people
	for(i=0; i<laser_legs.size(); i++)
		people_.push_back(Person(laser_legs[i], time));

	//Remaining positions are predicted
	for(i=0; i<people_aux.size(); i++)
	{
		people_aux[i].predict(time);
		people_.push_back(people_aux[i]);
	}

	return;
}
//Returns the projected 3D world point in the unrectified image
cv::Point2d PeopleDetector::point3dTo2d(tf::Point point3d)
{
	//Correct transform to follow camera coordinates system standard
	cv::Point3d pt_cv  (-1*(point3d.y() + 0.06), -1*point3d.z(), point3d.x());	
	cv::Point2d point_2d_rect = cam_model_.project3dToPixel(pt_cv);

	//Points are unrectified since we are using raw images
	return cam_model_.unrectifyPoint(point_2d_rect);
}

//Returns the index to the closest point in a vector given a maximum distance using 3D coordinates
int PeopleDetector::getClosestPoint(tf::Point point, vector<Person>& candidates,double max_dist)
{
	double dist, min_dist;
	int min_index = -1;

	for(int i=0; i<candidates.size(); i++)
	{
		dist = point.distance(candidates[i].getPos());
		if(dist <= max_dist && (min_index == -1 || candidates[i].getErrorCov() < candidates[min_index].getErrorCov()))
		{
			min_index = i;
			min_dist = dist;
		}
	}
	return min_index;
}


//OVERLOADED getClosesPoint
//Returns the index to the closest point in a vector given a maximum distance using 2d points on the image
int PeopleDetector::getClosestPoint(cv::Point2d laser_pt, vector<cv::Point2d>& image_pts,double max_dist)
{
	double dist, min_dist;
	int min_index = -1;

	for(int i=0; i<image_pts.size(); i++)
	{
		dist = cv::norm(laser_pt - image_pts[i]);
		if(dist <= max_dist && (min_index == -1 || dist < min_dist) )
		{
			min_index = i;
			min_dist = dist;
		}
	}
	return min_index;
}

void PeopleDetector::discardInactives()
{
	for(int i=0; i< people_.size(); i++)
	{
		//ros::Duration inactive_time = time - people_[i].getTime();
		if(people_[i].getFiab() < 0)
		{
			people_.erase(people_.begin()+ i--);
		}
	}
	return;
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

void PeopleDetector::mergeLaserAndImageData( vector<cv::Rect>& image_ROI, cv::Mat image, ros::Time time)
{
	int i, j;
	vector<cv::Point2d> cv_ppl_point, cv_image_point;
	vector<int> index;

	//Project 3D points to image
	for(i=0; i < people_.size(); i++)
	{
		tf::Point bottom_pt = tf_camera_base_link_.inverse() * (tf_laser_base_link_ * people_[i].getPos());
		cv_ppl_point.push_back( point3dTo2d(bottom_pt) );
	}

	//Get image ROI bottom middle point
	for(i=0; i < image_ROI.size(); i++)
		cv_image_point.push_back(cv::Point2d(image_ROI[i].x + (int) image_ROI[i].width/2, image_ROI[i].y + image_ROI[i].height));	

	//Calculate closest people position to merged data
	for(i=0; i < cv_ppl_point.size(); i++)
		index.push_back(getClosestPoint(cv_ppl_point[i], cv_image_point, max_img_dist_) );
	
	//Check first those person who share no other closest image detection
	for(i=0; i < index.size() && index[i] != -1; i++)
	{
		for(j=0; j<index.size(); j++)
			if(i!=j && index[i] == index[j])
				break;
		if(j == index.size())
		{
			//UPDATE PERSON 
			people_[i].addFiab();
			cv_image_point.erase(cv_image_point.begin() + index[i]);
			cv_ppl_point.erase(cv_ppl_point.begin() + i);
			index.erase(index.begin() + i--);
		}	
	}
	//Check the first pair of legs that share the same person
	bool pair_found = false;
	int closest_index;
	for(i=0; i<index.size() && index[i] != -1 && !pair_found; i++)
	{
		for(j=i+1; j<index.size() && !pair_found; j++)
		{
			if(index[i] == index[j])
			{
				closest_index = ( cv::norm(cv_ppl_point[i] - cv_image_point[index[i]]) < cv::norm(cv_ppl_point[j]- cv_image_point[index[i]])) ? i : j;
				//UPDATE PERSON
				people_[closest_index].addFiab();
				cv_image_point.erase(cv_image_point.begin() + index[i]);
				cv_ppl_point.erase(cv_ppl_point.begin() + closest_index);
				pair_found = true;
			}
		}
			
	}	
	//If we found a pair, we have to recalculate again the distance of the remaining legs
	if(pair_found)
		mergeLaserAndImageData(image_ROI, image, time);
	return;
}

/*
tf::Point PeopleDetector::ROIto3dPoint(cv::Rect ROI)
{
	//Projection will be done using the middle base point of the rectangle
	cv::Point2d bot_mid(ROI.x + (int) ROI.width/2, ROI.y + ROI.height);	
	cv::Point2d bot_mid_rect = cam_model_.rectifyPoint(bot_mid);
	
	//Get the ray projection. The unit vector we get is in the camera coordinate system.
	cv::Point3d unit_vect_cam = cam_model_.projectPixelTo3dRay(bot_mid_rect);
	//Convert from camera coordinate system to tf system
	tf::Point unit_vect(unit_vect_cam.z, -1*unit_vect_cam.x, -1*unit_vect_cam.y);

	double ground_distance = -1.26/unit_vect.z();

	return tf::Point(unit_vect.x()*ground_distance, unit_vect.y()*ground_distance,0);
}
*/
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

	marker_init.lifetime = ros::Duration(max_inactive_time_);
 
	for(int i=0; i<people_.size() && (people_[i].getErrorCov() >= sensitivity_) ; i++)
	{
		Marker marker = marker_init;
		//marker.header.stamp = odom_msg->header.stamp;	
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

void PeopleDetector::getLaserDetectionROI(vector<tf::Point>& laser_legs, vector<cv::Rect>& laser_ROI, cv::Mat image)
{
	vector<cv::Rect> ROI_rect;
	vector<tf::Point> legs_rect;

	for(int i=0; i<laser_legs.size(); i++)
	{
		//We have to convert the coordinates to the camera reference system and add ROI rectangle (0.5x1.80 m)
		tf::Point bottom_pt = tf_camera_base_link_.inverse() * (tf_laser_base_link_ *laser_legs[i]);

		cv::Point2d uv_botright = point3dTo2d(bottom_pt + tf::Point(0, 0.25, 0));
		cv::Point2d uv_topleft = point3dTo2d(bottom_pt + tf::Point(0,-0.25,1.80));

		cv::Rect ROI(uv_topleft,uv_botright);

		//Check whether the ROI is fully contained in the image
		cv::Rect rectsIntersection = ROI & cv::Rect(cv::Point(0,0),image.size());
		if((ROI & rectsIntersection) == ROI)
		{
	   		laser_ROI.push_back(ROI);
			legs_rect.push_back(laser_legs[i]);
		}
	}

	laser_legs = legs_rect;
	return;	
}

void PeopleDetector::getImageDetectionROI(vector<cv::Rect>& image_ROI, cv::Mat image)
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
		
		//Projection will be done using the middle base point of the rectangle
		cv::Point2d bot_mid(ROI_unfiltered[i].x + (int) ROI_unfiltered[i].width/2, ROI_unfiltered[i].y + ROI_unfiltered[i].height);	
		cv::Point2d bot_mid_rect = cam_model_.rectifyPoint(bot_mid);
	
		//Get the ray projection. The unit vector we get is in the camera coordinate system.
		cv::Point3d unit_vect_cam = cam_model_.projectPixelTo3dRay(bot_mid_rect);
		//Convert from camera coordinate system to tf system
		tf::Point unit_vect(unit_vect_cam.z, -1*unit_vect_cam.x, -1*unit_vect_cam.y);

		//Save only rectangles whose 3d ray projection goes through the floor (rectangles on upper positions cannot belong to real people)
		if(unit_vect.z() < 0)
			image_ROI.push_back(ROI_unfiltered[i]);
	}
	return;
}

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

