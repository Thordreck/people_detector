#ifndef PERSON_H
#define PERSON_H

#include "tf/tf.h"
#include "ros/ros.h"
#include "iostream"

#include "person_kf.h"

class Person
{
		static int			next_id;
		int				id_;			//Unique person id
		int				life_time_;		//If zero person is deleted
		ros::Time			last_update_time_;	//Time of the last prediction

		PersonKF			*kf_;


	public:
				Person(tf::Point pos, ros::Time time);
				Person(const Person& other);
				~Person();
		void		update(ros::Time time);		
		void		update(tf::Point pos, ros::Time time, bool img_detection);
		void		update(int cx, int cy, tf::Transform transform, ros::Time time);
		
		tf::Point	getPos();
		ros::Time	getTime();
		int		getId();
		int		getLifeTime();
		
		friend		std::ostream& operator<<(std::ostream &output, const Person& P);	
		Person& 	operator=(const Person& other);	
};

#endif
