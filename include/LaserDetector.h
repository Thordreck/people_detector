#ifndef LASER_DETECTOR_H
#define LASER_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include "LegsDetector.h"

class LaserDetector
{
		const int			MAX_LASER_DIST_;
		const double			MAX_LEGS_DIST_;
		double				threshold_;
		FILE				*f_hypotheses_;
		int				list_size_;
		LegsDetector			ld_;
		laser_geometry::LaserProjection	projector_;		
	public:
		LaserDetector(std::string hypotheses_filename, int num_hypotheses = 100, double threshold = 0.15, int list_size = 500);
		~LaserDetector();
		void scan_message(std::vector<tf::Point>& legs_points,const sensor_msgs::LaserScan::ConstPtr &msg);
			
};

#endif
