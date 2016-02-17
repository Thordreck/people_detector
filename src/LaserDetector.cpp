//standards
#include <iostream>
#include <float.h>

//People detector
#include "LaserDetector.h"
#include "LegsDetector.h"

//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;


LaserDetector::LaserDetector(string hypotheses_filename, int num_hypotheses /*= 100*/, double threshold /*= 0.15*/, int list_size /*= 500*/) 
 : MAX_LASER_DIST_(5), MAX_LEGS_DIST_(0.65), threshold_(threshold), list_size_(list_size)
{
	//Open hypotheses file
	f_hypotheses_ = fopen(hypotheses_filename.c_str(),"r");
	if (f_hypotheses_ == NULL)
		ROS_ERROR("ERROR opening hypothesis file %s", hypotheses_filename.c_str());
	ld_.load(f_hypotheses_, num_hypotheses);
	return;
}

LaserDetector::~LaserDetector()
{
    	fclose( f_hypotheses_ );
	return;
}
  
  /**
   * \brief getting range messages from your system and classify them
   * \param msg input laser scan
   */
void LaserDetector::scan_message(vector<tf::Point>& legs_points, const sensor_msgs::LaserScan::ConstPtr &msg)
{
	sensor_msgs::PointCloud cloud;
	projector_.projectLaser(*msg,cloud);

	// This is a list of segments.
   	// For more information have a look at ../common/dynamictable.h/hxx
    	dyntab_segments *list_segments=NULL;
    	int num_readings = msg->ranges.size();

    	double *angles = new double[num_readings]; // array of angles
    	double *ranges = new double[num_readings]; // array of measurements
    
     	for (unsigned int i = 0; i < num_readings; i++)
    	{
      		ranges[i] = msg->ranges[i];
      		angles[i] = msg->angle_min + i * msg->angle_increment;
    	}

    	list_segments = new dyntab_segments (list_size_);
    	// segment the scan
    	ld_.segmentScan(threshold_, num_readings, angles, ranges, list_segments);
		
    	// Classiy segments
    	for (int i=0; i < list_segments->num(); i++) 
    	{
      		Segment *s = list_segments->getElement(i);

      		// discard segments with less than three points
      		if ( s->num() < 3 ) 
        		s->type = -1;
      		else 
        		ld_.classify(s); 
	}

	//Associate legs
    	int left_leg_index = 0;
	int right_leg_index = 0;
	tf::Point left_leg, right_leg;


    	for (int i=0; i < list_segments->num(); i++) 
    	{
		Segment *s = list_segments->getElement(i);
		left_leg_index += s->num() -1;

      		if (s->type == 1 && ranges[left_leg_index] <= MAX_LASER_DIST_)
		{
			left_leg.setValue(cloud.points[left_leg_index].x, cloud.points[left_leg_index].y, 0);

			Segment *next_s = NULL;
			right_leg_index = left_leg_index +1; 

			int j;
			for(j=i+1; j < list_segments->num(); j++)
			{
				next_s = list_segments->getElement(j);
				if (next_s->type == 1)
				{
					right_leg.setValue(cloud.points[right_leg_index].x, cloud.points[right_leg_index].y, 0);
					break;
				}
				right_leg_index += next_s->num();
			}

			if(next_s && next_s->type == 1 && left_leg.distance(right_leg) <= MAX_LEGS_DIST_)
			{
				left_leg += right_leg;
				left_leg /= 2;
				i = j;
				left_leg_index = right_leg_index + next_s->num();
				legs_points.push_back(left_leg);	
			}
			else
			{
				legs_points.push_back(left_leg);
				i = j-1;
				left_leg_index = right_leg_index;
			}
		}
		else
			left_leg_index++;
	}

    	// delete the list of segments 
    	list_segments->setAutoDelete(true);
    	delete list_segments;
    	list_segments = NULL;

    	// free memory
    	delete [] ranges;
    	delete [] angles;

    	return;
}
//end LaserDetector

