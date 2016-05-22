#include "Person.h"
#include "person_kf.h"

using namespace std;
using namespace ros;
using namespace cv;

int Person::next_id;

Person::Person(tf::Point pos, Time time) : id_(++next_id)
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
	return;
}

//Update the position of the person when info from the detection is available
void Person::update(tf::Point pos, Time time)
{
	double dt = time.sec - last_update_time.sec;
	kf_->update(pos.x(), pos.y(), time);	
	last_update_time = time;
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

