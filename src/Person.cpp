#include "Person.h"

using namespace std;
using namespace ros;
using namespace cv;



int Person::next_id;

Person::Person(tf::Point pos,Time time) : id_(++next_id),  position_(pos), velocity_(0,0,0), fiability_(10)
{

	//Kalman filter initialization
	//Kalman filter parameters
	//	4 dinamic parameters: x,y,v_x,v_y
	//	2 measure parameters: x,y
	//	0 control parameters
	kf_ = new KalmanFilter(4,2,0);

	//Transition matrix A. We assume people's velocity is constant and dt will be updated on each iteration
	//[1 0 dt 0
	// 0 1 0  dt
	// 0 0 1  0
	// 0 0 0  1]
	//dt is initially set to 0
	kf_->transitionMatrix = (Mat_<float>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

	//Measurement matrix H is set to identity
	setIdentity(kf_->measurementMatrix);
	
	//Noise matrices initialization

	//Q: Process noise covariance is set to 0.0225, meaning that given a prediction, we are fairly sure most of the time our target will be
	//in +- 0.3m from the position (average step size)
	//EXPLANATION
	//Laser rate ~= 7.7hz ; Camera rate ~= 4.5hz. Taking the minimun rate gives as a period of 0.22 seconds between measures. 
	//Taking the average speed
	//of an human adult as 5km/h = 1.39m/s gives us an average variation of +-0.3m in the position between measurements.
	//Q matrix gives a value of how fast the measured signal varies
	setIdentity(kf_->processNoiseCov, Scalar::all(0.0225));

	//R: Measurement noise covariance is set to 0.2 (see covariance in pose messages)
	//R matrix gives an estimation of how noisy is our measurements. If it is high compared to Q values will be more heavily filtered
	setIdentity(kf_->measurementNoiseCov, Scalar::all(2*1e-1));
	
	//We are sure our initial inputs are within sqrt(0.01) = +-0.1m 
	setIdentity(kf_->errorCovPost, Scalar::all(1e-2));

	//Initial states set to position detected and velocity zero
	//Set initial state
	kf_->statePre.at<float>(0) = position_.x();
	kf_->statePre.at<float>(1) = position_.y();
	kf_->statePre.at<float>(2) = 0;
	kf_->statePre.at<float>(3) = 0;

	//Set the post state equal to initial state for the initialization
	kf_->statePost.at<float>(0) = position_.x();
	kf_->statePost.at<float>(1) = position_.y();
	kf_->statePost.at<float>(2) = 0;
	kf_->statePost.at<float>(3) = 0;

	
	last_correct_time_ = time;
	last_predict_time_ = time;

	return;
}

//Copy constructor
Person::Person(const Person &psource) : id_(psource.id_),  position_(psource.position_), velocity_(psource.velocity_), last_correct_time_(psource.last_correct_time_), last_predict_time_(psource.last_predict_time_), kf_(psource.kf_), meas(psource.meas) 
{
	return;
}

void Person::correct(tf::Point pos, Time time)
{
	updateTime(time,last_correct_time_);
	last_correct_time_ = time;
	last_predict_time_ = time;

	//Prediction is run before correction 
	kf_->predict();
	//This prevents the filter from getting stuck in a state when no corrections are executed
	kf_->statePre.copyTo(kf_->statePost);
	kf_->errorCovPre.copyTo(kf_->errorCovPost);

	setData(kf_->correct((Mat_<float>(2,1) << pos.x(), pos.y())), time);
	return;	
}

void Person::predict(ros::Time time)
{
	//Before predicting we update the dt values of the transition matrix
	updateTime(time,last_predict_time_);	
	last_predict_time_ = time;

	setData(kf_->predict(),time);
	
	//This prevents the filter from getting stuck in a state when no corrections are executed
	kf_->statePre.copyTo(kf_->statePost);
	kf_->errorCovPre.copyTo(kf_->errorCovPost);
	return;
}

inline void Person::updateTime(ros::Time time, ros::Time last_time)
{
	int dt = time.sec - last_time.sec;
	kf_->transitionMatrix.at<int>(2) = dt;
	kf_->transitionMatrix.at<int>(7) = dt;
	return;
}

inline void Person::setData(const Mat& meas, Time time)
{
	position_ = tf::Point(meas.at<float>(0), meas.at<float>(1),0);
	velocity_ = tf::Vector3(meas.at<float>(2),meas.at<float>(3),0);
	return;
}

ostream& operator<<(ostream& output, const Person &P)
{
	output << "Person nº" << P.id_ << endl;
	output << "X= " << P.position_.x() << endl << "Y= " << P.position_.y() << endl;
	output << "V_x= " << P.velocity_.x() << endl << "V_y= " << P.velocity_.y() << endl;
	output << "ERROR COV POST =" << P.kf_->errorCovPost << endl;
	output << "ERROR COV PRE =" << P.kf_->errorCovPre << endl;
	return output;
}

tf::Point Person::getPos()
{
	return position_;
}

Time Person::getTime()
{
	return last_correct_time_;
}
int Person::getId()
{
	return id_;
}

float Person::getErrorCov()
{
	return kf_->errorCovPost.at<float>(0,0);
}

int Person::getFiab()
{
	return fiability_;
}

void Person::addFiab()
{
	fiability_++;
	return;
}

void Person::subFiab()
{
	fiability_--;
	return;
}
