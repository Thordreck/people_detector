#ifndef __PERSON_KF_H__
#define __PERSON_KF_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
//#include <Eigen/LU>

#define PERSONKF_LASER_POS_VAR 0.1
#define PERSONKF_IMAGE_X_VAR 0.1
#define PERSONKF_IMAGE_Y_VAR 0.1
#define PERSONKF_VEL_NOISE_VAR 0.2  

//Anonymous namespace is used to avoid multiple definition compiler errors when calling PersonKF static functions from other files
namespace{
struct PersonKF
{
	// State vector: [x (m), y (m), vx (m/s), vy (m/s)]
	Eigen::MatrixXd x;
	Eigen::MatrixXd P;
	
	//Camera projection matrix K
	static Eigen::Matrix3d K_cam;

	// Last time update
	ros::Time tStamp;
	
	// Updated flag
	bool updated;

	
	// Default constructor
	PersonKF(void) : x(4,1), P(4,4) 
	{
		x.setZero(4, 1);
		P.setIdentity(4, 4);
		tStamp = ros::Time::now();
		updated = false;
	}
	
	PersonKF(const PersonKF &data) : x(4,1), P(4,4)
	{
		x = data.x;
		P = data.P;
		tStamp = data.tStamp;
		updated = data.updated;
	}
	
	PersonKF &operator=(const PersonKF &data)
	{
		x = data.x;
		P = data.P;
		tStamp = data.tStamp;
		updated = data.updated;
		return *this;
	}
	
	// Filter initialization, position in m and velocity on m/s 
	void init(double _x, double _y, double _vx, double _vy, ros::Time _tStamp)
	{

		// Setup state vector
		x.setZero(4, 1);
		x(0,0) = _x;
		x(1,0) = _y;
		x(2,0) = _vx;
		x(3,0) = _vy;
		
		// Setup cov matrix
		P.setIdentity(4, 4);
		P(0,0) = PERSONKF_LASER_POS_VAR;
		P(1,1) = PERSONKF_LASER_POS_VAR;
		P(2,2) = 1.0*1.0;
		P(3,3) = 1.0*1.0;
		
		// Update time stamp
		tStamp = _tStamp;
		updated = false;
	}

	static void set_caminfo(double _fx, double _fy, double _u, double _v);

	// State prediction, time in seconds 
	void predict(double _dt)
	{
		// State vector prediction
		x(0,0) += x(2,0)*_dt;
		x(1,0) += x(3,0)*_dt;
		
		// Convariance matrix prediction
		Eigen::Matrix<double, 4, 4> F;
		F.setIdentity(4, 4);
		F(0,2) = _dt;
		F(1,3) = _dt;
		Eigen::Matrix<double, 4, 4> Q;
		Q.setZero(4, 4);
		Q(2,2) = PERSONKF_VEL_NOISE_VAR*_dt*_dt;
		Q(3,3) = PERSONKF_VEL_NOISE_VAR*_dt*_dt;
		P = F*P*F.transpose() + Q;
		
		updated = false;
	}
	
	// State update for laser data
	void update(double _x, double _y, ros::Time _t)
	{
		// Update time stamp
		tStamp = _t;
		
		// Compute update jacobian
		Eigen::Matrix<double, 2, 4> H;
		H.setZero(2, 4);
		H(0,0) = 1.0;
		H(1,1) = 1.0;
		
		// Compute update noise matrix
		Eigen::Matrix<double, 2, 2> R;
		R.setZero(2, 2);
		R(0,0) = PERSONKF_LASER_POS_VAR;
		R(1,1) = PERSONKF_LASER_POS_VAR;
		
		// Calculate innovation matrix
		Eigen::Matrix<double, 2, 2> S;
		S = H*P*H.transpose() + R;
		
		// Calculate kalman gain
		Eigen::Matrix<double, 4, 2> K;
		K = P*H.transpose()*S.inverse();
		
		// Calculate innovation vector
		Eigen::Matrix<double, 2, 1> y;
		y(0,0) = _x - x(0,0);
		y(1,0) = _y - x(1,0);
		
		// Calculate new state vector
		x = x + K*y;
		
		// Calculate new cov matrix
		Eigen::Matrix<double, 4, 4> I;
		I.setIdentity(4, 4);
		P = (I - K*H)*P;
		
		updated = true;
	}
	
	// State update for image data
	void update(int _cx, int _cy, Eigen::Matrix<double, 3, 4> _TF, ros::Time _t)
	{
		// Update time stamp
		tStamp = _t;

		// Compute update jacobian
		Eigen::Matrix<double, 2, 4> H;
		H.setZero(2, 4);
		H(0,0) =  (_TF(0,0)*(K_cam(0,0)*_TF(1,3) - _TF(0,3)*K_cam(0,2) + x(0,0)*(K_cam(0,0)*_TF(1,0) - _TF(0,0)*K_cam(0,2)) + x(1,0)*(K_cam(0,0)*_TF(1,1) - _TF(0,1)*K_cam(0,2))))/((_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))*(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))) - (K_cam(0,0)*_TF(1,0) - _TF(0,0)*K_cam(0,2))/(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0)); 

		H(0,1) =  (_TF(0,1)*(K_cam(0,0)*_TF(1,3) - _TF(0,3)*K_cam(0,2) + x(0,0)*(K_cam(0,0)*_TF(1,0) - _TF(0,0)*K_cam(0,2)) + x(1,0)*(K_cam(0,0)*_TF(1,1) - _TF(0,1)*K_cam(0,2))))/((_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))*(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))) - (K_cam(0,0)*_TF(1,1) - _TF(0,1)*K_cam(0,2))/(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0)); 

		H(1,0) =  (_TF(0,0)*(K_cam(1,1)*_TF(2,3) - _TF(0,3)*K_cam(1,2) + x(0,0)*(K_cam(1,1)*_TF(2,0) - _TF(0,0)*K_cam(1,2)) + x(1,0)*(K_cam(1,1)*_TF(2,1) - _TF(0,1)*K_cam(1,2))))/((_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))*(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))) - (K_cam(1,1)*_TF(2,0) - _TF(0,0)*K_cam(1,2))/(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0)); 

		H(1,1) =  (_TF(0,1)*(K_cam(1,1)*_TF(2,3) - _TF(0,3)*K_cam(1,2) + x(0,0)*(K_cam(1,1)*_TF(2,0) - _TF(0,0)*K_cam(1,2)) + x(1,0)*(K_cam(1,1)*_TF(2,1) - _TF(0,1)*K_cam(1,2))))/((_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))*(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0))) - (K_cam(1,1)*_TF(2,1) - _TF(0,1)*K_cam(1,2))/(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0)); 

		// Compute update noise matrix
		Eigen::Matrix<double, 2, 2> R;
		R.setZero(2, 2);
		R(0,0) = PERSONKF_IMAGE_X_VAR;
		R(1,1) = PERSONKF_IMAGE_Y_VAR;
		
		// Calculate innovation matrix
		Eigen::Matrix<double, 2, 2> S;
		S = H*P*H.transpose() + R;
		
		// Calculate kalman gain
		Eigen::Matrix<double, 4, 2> K;
		K = P*H.transpose()*S.inverse();
		
		// Calculate innovation vector
		Eigen::Matrix<double, 2, 1> y;
		Eigen::Matrix<double, 2, 1> c;
		c(0,0) = _cx;
		c(1,0) = _cy;

		Eigen::Matrix<double, 2, 1> c_est;
		c_est(0,0) = (-1*(K_cam(0,0)*_TF(1,3) - _TF(0,3)*K_cam(0,2) + x(0,0)*(K_cam(0,0)*_TF(1,0) - _TF(0,0)*K_cam(0,2)) + x(1,0)*(K_cam(0,0)*_TF(1,1) - _TF(0,1)*K_cam(0,2))))/(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0)); 
		c_est(1,0) = (-1*(K_cam(1,1)*_TF(2,3) - _TF(0,3)*K_cam(1,2) + x(0,0)*(K_cam(1,1)*_TF(2,0) - _TF(0,0)*K_cam(1,2)) + x(1,0)*(K_cam(1,1)*_TF(2,1) - _TF(0,1)*K_cam(1,2))))/(_TF(0,3) + _TF(0,0)*x(0,0) + _TF(0,1)*x(1,0)); 

		//y = c - h*x; 
		y = c - c_est; 
		
		// Calculate new state vector
		x = x + K*y;
		
		// Calculate new cov matrix
		Eigen::Matrix<double, 4, 4> I;
		I.setIdentity(4, 4);
		P = (I - K*H)*P;
		
		updated = true;
	}
};

Eigen::Matrix3d PersonKF::K_cam;

//Set camera projection matrix K
void PersonKF::set_caminfo(double _fx, double _fy, double _u, double _v)
{
	K_cam.setZero(3, 3);
	K_cam(0,0) = _fx;
	K_cam(0,2) = _u;
	K_cam(1,1) = _fy;
	K_cam(1,2) = _v;
	K_cam(2,2) = 1.0;
	return;
}
}
#endif
