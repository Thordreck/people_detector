#ifndef PERSON_H
#define PERSON_H

#include "opencv2/opencv.hpp"
#include "tf/tf.h"
#include "ros/ros.h"
#include "iostream"

class Person
{
		static int			next_id;
		int				id_;			//Unique person id

		ros::Time			last_correct_time_,last_predict_time_;	//Time of the last prediction

		tf::Point			position_;		//Person position in /base_link frame
		tf::Vector3			velocity_;

		cv::KalmanFilter		*kf_;			//Kalman filter used to predict position and velocity
		cv::Mat				meas;
		
		int				fiability_;

		void				updateTime(ros::Time time, ros::Time last_time); 
		void				setData(const cv::Mat& meas, ros::Time time);
	public:
						Person(tf::Point pos, ros::Time time);
						Person(const Person &psource);
		void 				predict(ros::Time time);
		void				correct(tf::Point pos, ros::Time time);	
		
		friend				std::ostream &operator<<(std::ostream &output, const Person &P);	
		tf::Point			getPos();
		ros::Time			getTime();
		int				getId();
		float				getErrorCov();
		int				getFiab();
		void				addFiab();
		void				subFiab();
};

#endif
