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


PeopleDetector::PeopleDetector() : MIN_INTERSECT_(0.3), MAX_STEP_(0.6)
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

	//Flags
	cam_info_set_ = false;
	static_tf_set_ = false;
	global_tf_set_ = false;

	//Give time to tf listener to initialize before using it
	ros::Duration duration(3.0);
	duration.sleep();

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

	//Set camera info data for kalman fiter
	if(!cam_info_set_)
	{
		PersonKF::set_caminfo(info_msg->K[0], info_msg->K[2], info_msg->K[4], info_msg->K[5]);
		cam_info_set_ = true;
	}

	//Set static info transforms
	if(!static_tf_set_)
	{
		try
		{
        		tf_listener_->lookupTransform("/bumblebee",  "/base_link", ros::Time(0), tf_base_camera_);
			tf_camera_base_.setData(tf_base_camera_.inverse());
			static_tf_set_ = true;
        	}
    		catch (tf::TransformException ex)
		{
        		ROS_WARN("Cannot get static transform: %s", ex.what());
        	}
	}

	global_tf_set_ = false;
	try{
		tf_listener_->lookupTransform("/odom", "/base_link", time, tf_base_global_);
		tf_global_base_.setData(tf_base_global_.inverse());
		global_tf_set_ = true;
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		global_tf_set_ = false;
	}

	vector<tf::Point> legs_front, legs_back;
	vector<cv::Rect> image_ROI, laser_ROI;

	//Extract image from ros message and convert it to openCV Mat format
	cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

	//Get camera info for 3D projection onto images
	cam_model_.fromCameraInfo(info_msg);

	//Run both image and laser based person detector
	ld_->scan_message(legs_front, laserfront_msg);
	ld_->scan_message(legs_back, laserback_msg);
	getLaserDetection(legs_front, legs_back, laser_ROI, laserfront_msg->header.stamp);
	getImageDetection(image_ROI, image);

	for(int i=0; i<laser_ROI.size(); i++)
		cv::rectangle(image,laser_ROI[i], cv::Scalar(0,0,255), 2);	

	for(int i=0; i<image_ROI.size(); i++)
		cv::rectangle(image,image_ROI[i], cv::Scalar(255,0,0), 2);	


	//Merge laser and image data
	vector<mergedData> merged_data;

	mergeData(merged_data, image_ROI, laser_ROI, legs_front, legs_back);

	for(int i=0; i<merged_data.size(); i++)
		cv::rectangle(image,merged_data[i].ROI, cv::Scalar(0,255,0), 1);	

	//Update people data
	updatePeople(merged_data, time);

	//Remove people who have not been updated for a while and their lifetime has reached zero
	//TODO: avoid using erase method
	for(int i=0; i<people_.size(); i++)
	{
		if(people_[i].getLifeTime() <= 0 )
			people_.erase(people_.begin() + i--);
	}

	//Publish detected people
	publishPeople();
	
	//printPeople(image, time);

	cv::imshow("Frame", image);
	cv::waitKey(1);
	cout << "TIEMPO: " << t.elapsed() << endl;
	cout << "------------------------------" << endl;

	return;
}


void PeopleDetector::updatePeople(vector<mergedData>& merged_data, ros::Time time)
{
	//TODO: unify updatePeople and mergeData functions code in single general function, since the merge algorightm is the same

	int merge_index; 	//ROI laser index to merge with current image data
	double merge_area;  	//Intersection area of merge candidate
	double merge_dist;	//Distance to merge candidate

	for(int i=0; i < people_.size(); i++)
	{
		merge_index = -1;
		merge_area = 0.0;
		merge_dist = 100.0;
		//Get person ROI: first we get the position of the person in global coordinates, 
		//transform them into base_link coordinates and get the ROI
		tf::Point p1 = globalCoordinatesToLocal( people_[i].getPos(), people_[i].getTime());
		cv::Rect r1 = point3dToROI( p1, people_[i].getTime() );

		//Check if there is a projection fully contained in the image
		if( r1.area() )
		{
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
		}
		//If there is not projection onto the image we merge the data using distance measurements
		else
		{
			for(int j=0; j < merged_data.size(); j++)
			{
				if(merged_data[j].ROI.area())
					continue;
				double dist = p1.distance(merged_data[j].pos);
				if(dist < MAX_STEP_ && dist < merge_dist)
				{
					merge_index = j;
					merge_dist = dist;
				}
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
				//TODO: use a pixel a bit higher 
				int cx = data.ROI.x + (int) data.ROI.width/2;
				int cy = data.ROI.y + data.ROI.height;					
				
				if(global_tf_set_)
					people_[i].update(cx, cy, tf_base_camera_*tf_global_base_, time);
				else
					people_[i].update(time);
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
		{
			people_.push_back( Person( localCoordinatesToGlobal(data.pos, time), time) );
		}
	}
	return;
}


//Returns the projected 3D world point in base_link frame in the unrectified image
//Returns 0 if the 3d point is located behind the camera
cv::Point2d PeopleDetector::point3dTo2d(tf::Point point3d)
{
	//Correct transform to follow camera coordinates system standard
	//TODO: fix tf publisher and use camera info messages to get distance between stereo cameras
	cv::Point3d pt_cv  (-1*(point3d.y() + 0.06), -1*point3d.z(), point3d.x());	
	cv::Point2d point_2d_rect;
	//If the point is behind the camera
	if(pt_cv.z <= 0)
		point_2d_rect = cv::Point2d(0,0);
	else
		point_2d_rect = cam_model_.project3dToPixel(pt_cv);

	//Points are unrectified since we are using raw images
	return cam_model_.unrectifyPoint(point_2d_rect);
}

//Returns ROI of person using 3D point on base_link frame
//If the ROI is not fully contained in the image, it returns an empty ROI (Size(0,0))
cv::Rect PeopleDetector::point3dToROI(tf::Point point3d, Time time)
{

	cv::Point2d uv_botright, uv_topleft;
	
	if(static_tf_set_)
	{
		tf::Point bottom_pt = tf_base_camera_ * point3d;
		uv_botright = point3dTo2d(bottom_pt + tf::Point(0, 0.25, 0));
		uv_topleft = point3dTo2d(bottom_pt + tf::Point(0,-0.25,1.80));
	
		cv::Rect ROI(uv_topleft, uv_botright);
	
		//Check if the ROI is fully contained in the image
		cv::Rect rectsIntersection = ROI & cv::Rect(cv::Point(0,0),cam_model_.fullResolution());
		if( (ROI & rectsIntersection) == ROI )
			return ROI;
	}
	return cv::Rect();
}

//Transform from robot relative coordinates to global coordinates using odometry info
tf::Point PeopleDetector::localCoordinatesToGlobal(tf::Point local_point, ros::Time time)
{
	if(!global_tf_set_)
		return local_point;
	
	return tf_base_global_ * local_point;
}

//Transform from robot relative coordinates to global coordinates using odometry info
tf::Point PeopleDetector::globalCoordinatesToLocal(tf::Point global_pt, ros::Time time)
{
	if(!global_tf_set_)
		return global_pt;
	
	return tf_global_base_ * global_pt;
}

//Merge laser and image data. The merge is undergone when laser and image ROI detection intersects with a percentage high enough. As image detection ROI are usually bigger than
//laser detection ROIs, they take precedence.
void PeopleDetector::mergeData(vector<mergedData>& v_merged_data, vector<cv::Rect>& image_ROI, vector<cv::Rect>& laser_ROI, vector<tf::Point>& laser_legs, vector<tf::Point>& legs_back)
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

	//Legs with ROI not contained in the image are also added as merged data
	for(int i=0; i < legs_back.size(); i++)
	{
		merged_data.ROI = cv::Rect();
		merged_data.pos = legs_back[i];
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
	marker_init.type = Marker::CUBE;
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
void PeopleDetector::getLaserDetection(vector<tf::Point>& legs_front, vector<tf::Point>& legs_back, vector<cv::Rect>& legs_ROI, Time time)
{
	vector<tf::Point> legs_front_unfiltered, legs_front_filtered;
	vector<cv::Rect> legs_ROI_unfiltered;

	legs_front_unfiltered = legs_front;
	legs_front.clear();

	for(int i=0; i< legs_front_unfiltered.size(); i++)
	{
		cv::Rect ROI = point3dToROI(legs_front_unfiltered[i], time);

		//Check if there is an actual ROI. If not treat the 3d point as a back leg
		if( ROI.area() )
		{
			legs_ROI_unfiltered.push_back(ROI);
			legs_front_filtered.push_back(legs_front_unfiltered[i]);
		}
		else
			legs_back.push_back(legs_front_unfiltered[i]);
	}
	//Remove ROIs that are fully contained inside bigger ROIs
	int i,j;
	for(i=0; i< legs_ROI_unfiltered.size(); i++)
	{
		cv::Rect r = legs_ROI_unfiltered[i];

		for(j = 0; j< legs_ROI_unfiltered.size(); j++)
		{
			if(j!=i && (r & legs_ROI_unfiltered[j]) == r)
				break;
		}
		if(j == legs_ROI_unfiltered.size())
		{
			legs_ROI.push_back(legs_ROI_unfiltered[i]);
			legs_front.push_back(legs_front_filtered[i]);
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
