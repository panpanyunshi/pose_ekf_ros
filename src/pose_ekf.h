#ifndef __POSE_EKF_H
#define __POSE_EKF_H
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

//ekf state 
//0-3 quaternion
//4-6 Pn Pe Pd
//7-9 Vn Ve Vd
//10-12 bwx bwy bwz
//13-15 bax bay baz 

class Pose_ekf
{
public:
	Pose_ekf();
	~Pose_ekf();	
	void predict(Vector3d gyro, Vector3d acc, double t);
	void correct(Vector3d pos, Vector3d vel, Vector3d mag, double t);
	VectorXd process(Vector3d gyro, Vector3d acc, VectorXd& xdot, MatrixXd& F);
	MatrixXd computeF(Vector3d gyro, Vector3d acc);
	
	VectorXd measurement(VectorXd x, Vector3d mag);
	MatrixXd computeH(Vector3d mag);

	void measurement_fix(Vector2d& position, MatrixXd &H);
	void measurement_fix_velocity(Vector3d& velocity, MatrixXd& H);
	void measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H);


	void correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R);
	void correct_fix(Vector3d position, double t);
	void correct_fix_velocity(Vector3d velocity, double t);
	void correct_sonar_height(double sonar_height, double t);//todo, without considering the roll and pitch

	// void measurement_altimeter(double& altimeter_height, MatrixXd H);
	void getState(Quaterniond& q, Vector3d& position, Vector3d& velocity, Vector3d & bw, Vector3d&  ba);

private:
	VectorXd x;//state 
	MatrixXd P;//covariance

	//covariance parameter
	double position_cov;
	double height_cov;
	double velocity_cov;
	double gyro_cov;
	double acc_cov;
	double mag_cov;

	MatrixXd R; //measurement noise
	MatrixXd Q; //process noise

	MatrixXd R_fix;
	MatrixXd R_fix_velocity;
	MatrixXd R_sonar_height;

	Vector3d acc;
	Vector3d gyro;

	double current_t;
	bool initialized;

	bool fix_initialized;
	bool imu_initialized;
	bool altimeter_initialized;
	bool sonar_initialized;

	const int n_state = 16;
};

#endif 