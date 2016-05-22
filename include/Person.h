#ifndef PERSON_H
#define PERSON_H

#include "opencv2/opencv.hpp"
#include "tf/tf.h"
#include "ros/ros.h"
#include "iostream"

#include "person_kf.h"

class Person
{
		static int			next_id;
		int				id_;			//Unique person id
		ros::Time			last_update_time;	//Time of the last prediction

		PersonKF			*kf_;


	public:
				Person(tf::Point pos, ros::Time time);
				~Person();
		void		update(ros::Time);		
		void		update(tf::Point pos, ros::Time time);
		
		tf::Point	getPos();
		int		getId();
		
		friend		std::ostream &operator<<(std::ostream &output, const Person &P);	
};

#endif
