//standards
#include <iostream>
#include <float.h>

//People detector
#include "LaserDetector.h"
#include "LegsDetector.h"

//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;


LaserDetector::LaserDetector(string hypotheses_filename, int num_hypotheses /*= 100*/, double threshold_ /*= 0.15*/, int list_size_ /*= 500*/) 
 : max_range_(5)
{
	threshold = threshold_;
	list_size = list_size_;

	//Open hypotheses file
	f_hypotheses = fopen(hypotheses_filename.c_str(),"r");
	if (f_hypotheses == NULL)
		ROS_ERROR("ERROR opening hypothesis file %s", hypotheses_filename.c_str());
	ld.load(f_hypotheses, num_hypotheses);
	return;
}

LaserDetector::~LaserDetector()
{
    	fclose( f_hypotheses );
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
    list_segments = new dyntab_segments (list_size);
    // segment the scan
    ld.segmentScan(threshold, num_readings, angles, ranges, list_segments);
		

    int readings_index = 0;

    // Classiy segments
    for (int i=0; i < list_segments->num(); i++) 
    {
      Segment *s = list_segments->getElement(i);
	if(s->num() >=0 )
		readings_index += s->num()-1;
      // discard segments with less than three points
      if ( s->num() < 3 ) 
        s->type = -1;
      else 
        ld.classify(s); 
      
      if (s->type == 1 /*&& ranges[readings_index] <= max_range_*/)
      {
	legs_points.push_back(tf::Point(cloud.points[readings_index-(int)(s->num()/2)].x,cloud.points[readings_index-(int)(s->num()/2)].y,0));
      }
}

    // delete the list of segments 
    list_segments->setAutoDelete(true);
    delete list_segments;
    list_segments = NULL;

    // free memory
    delete [] ranges;
    delete [] angles;

	//------------------------------------

    return;
  }
  //end LaserDetector

