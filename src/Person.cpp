#include "Person.h"
#include "person_kf.h"

#include <Eigen/Core>

using namespace std;
using namespace ros;

int Person::next_id;

Person::Person(tf::Point pos, Time time) : id_(++next_id), life_time_(3)
{
	
	kf_ = new PersonKF;

	kf_->init(pos.x(), pos.y(), 0.0, 0.0, time);
	last_update_time = time;

	return;
}

Person::~Person()
{
	delete kf_;
	return;
}

//Use the kalman filter to update the position of the person when no info is available
void Person::update(Time time)
{
	double dt = time.sec - last_update_time.sec;
	kf_->predict(dt);	
	last_update_time = time;

	life_time_--;
	return;
}

//Update the position of the person when info from the detection is available
void Person::update(tf::Point pos, Time time, bool img_detection)
{
	double dt = time.sec - last_update_time.sec;
	kf_->update(pos.x(), pos.y(), time);	
	last_update_time = time;

	life_time_ = (img_detection) ? 5 : 3;
	return;
}

void Person::update(int cx, int cy, tf::StampedTransform transform, Time time)
{
	double dt = time.sec - last_update_time.sec;
	Eigen::Matrix<double, 3, 4> tf_matrix;
	
	//Copy rotation matrix
	tf::Matrix3x3 rotation = transform.getBasis();
	for(int i=0; i < 3; i++)
	{
		tf::Vector3 row = rotation.getRow(i);
		tf_matrix(i,0) = row.x();	
		tf_matrix(i,1) = row.y(); 	
		tf_matrix(i,2) = row.z(); 	
	}

	//Copy translation matrix
	tf::Vector3 translation = transform.getOrigin();

	tf_matrix(0,3) = translation.x();	
	tf_matrix(1,3) = translation.y();	
	tf_matrix(2,3) = translation.z();	

	kf_->predict(dt);	
	kf_->update(cx, cy, tf_matrix, time);	
	last_update_time = time;

	life_time_ = 5;
	return;
}

ostream& operator<<(ostream& output, const Person &P)
{

	output << "Person nº" << P.id_ << endl;
	output << "X= " << P.kf_->x(0,0) << endl << "Y= " << P.kf_->x(1,0) << endl;
	output << "V_x= " << P.kf_->x(2,0) << endl << "V_y= " << P.kf_->x(3,0) << endl;
	return output;
}

tf::Point Person::getPos()
{
	return tf::Point(kf_->x(0,0), kf_->x(1,0), 0.0);
}

int Person::getId()
{
	return id_;
}

int Person::getLifeTime()
{
	return life_time_;
}

Time Person::getTime()
{
	return last_update_time;
}
