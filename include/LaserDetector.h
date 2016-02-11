#ifndef LASER_DETECTOR_H
#define LASER_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include "LegsDetector.h"

class LaserDetector
{
		const int			max_range_;
		double				threshold;
		FILE				*f_hypotheses;
		int				list_size;
		LegsDetector			ld;
		tf::TransformListener		listener_;
		laser_geometry::LaserProjection	projector_;		
	public:
		LaserDetector(std::string hypotheses_filename, int num_hypotheses = 100, double threshold_ = 0.15, int list_size_ = 500);
		~LaserDetector();
		void scan_message(std::vector<tf::Point>& legs_points,const sensor_msgs::LaserScan::ConstPtr &msg);
			
};

#endif
