#include "pose_ekf.h"
#include <iostream>
#include <Eigen/Dense>
#include "conversion.h"
using namespace std;
using namespace Eigen;

#define eps 1e-6
//quaternion: body fram to navigation frame
//Rnb
//ekf state 
//0-3 quaternion
//4-6 Pn Pe Pd
//7-9 Vn Ve Vd
//10-12 bwx bwy bwz
//13-15 bax bay baz 

Vector4d productMatrix(Quaterniond q)
{
	Vector4d p;
	p(0) = q.w(); p.tail(3) = q.vec();
	MatrixXd P(4, 4);
	P << p(0), -p(1), -p(2), -p(3),
		 p(1), p(0), -p(3), p(2),
		 p(2), p(3), p(0), -p(1),
		 p(3), -p(2), p(1), p(0);
 	return P;
}

Matrix3d skew_symmetric(Vector3d v)
{
	Matrix3d m;
	m << 0, -v(2), v(1),
		 v(2), 0,  -v(0),
		 -v(1), v(0), 0;
	return m;
}

//diff_(p*q) /diff_q
Matrix4d diff_pq_q(Quaterniond p)
{
	double p0 = p.w();
	Vector3d pv = p.vec();

	Matrix4d D;
	D(0, 0) = p0;
	D.block<1, 3>(0, 1) = -pv.transpose();
	D.block<3, 1>(1, 0) = pv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*p0 + skew_symmetric(pv);
	return D;
}


//diff_(p*q)/ diff_p
Matrix4d diff_pq_p(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix4d D;
	D(0, 0) = q0;
	D.block<1, 3>(0, 1) = -qv.transpose();
	D.block<3, 1>(1, 0) = qv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*q0 - skew_symmetric(qv);
	return D;
}

//diff_(q*v*q_star)/ diff_q
MatrixXd diff_qvqstar_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v + skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() - q0*skew_symmetric(v));
	return D; 
}
//diff_(q*v*q_star)/ diff_v
Matrix3d diff_qvqstar_v(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix3d D;
	D = (q0*q0 - qv.dot(qv))*Matrix3d::Identity() + 2*qv*qv.transpose() + 2*q0*skew_symmetric(qv);
	return D; 
}

Pose_ekf::Pose_ekf() : position_cov(0.5), height_cov(2.0), velocity_cov(2.5), gyro_cov(0.0001), acc_cov(0.01), mag_cov(0.1)
{	
	initialized = false;
	x = VectorXd::Zero(16);
	x.head(4) << 1, 0, 0, 0;
	P = MatrixXd::Zero(16, 16);
	for (int i = 0; i < 16; ++i)
	{
		P(i, i) = 100.0;
	}

	Q = MatrixXd::Identity(16, 16)*0.1;
	R = MatrixXd::Identity(9, 9);
	R.block<3, 3>(0, 0) = Matrix3d::Identity()*mag_cov;
	R.block<3, 3>(3, 3) = Matrix3d::Identity()*position_cov;
	R(5, 5) = height_cov;
	R.block<3, 3>(6, 6) = Matrix3d::Identity()*velocity_cov;
}
Pose_ekf::~Pose_ekf()
{
	

}


void Pose_ekf::predict(Vector3d gyro, Vector3d acc, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	double dt = t - current_t;
	VectorXd xdot;
	MatrixXd F;
	process(gyro, acc, xdot, F);

	x += xdot*dt;
	F = MatrixXd::Identity(n_state, n_state) + F*dt;//continous F and discrete F
	
	P = F*P*F.transpose() + Q;//Q and t
	x.head(4).normalize();

	this->current_t = t;
	this->acc = acc;
	this->gyro = gyro;
}



//TODO 
// void Pose_ekf::correct(Vector3d pos, Vector3d vel, Vector3d mag, double t)
// {
//     if(!initialized)
// 	{
// 		initialized = true;
// 		this->current_t = t;
// 		return;
// 	}
// 	VectorXd z = VectorXd::Zero(9);
// 	z.head(3) = mag;
// 	z.segment<3>(3) = pos;
// 	z.tail(3) = vel;

//     VectorXd zhat = measurement(x, mag);
//     MatrixXd H = computeH(mag);
//     cout << "z: " << z.transpose() << endl;
//     cout << "zhat: " << zhat.transpose() << endl;
//     cout << "dz: " << (z - zhat).transpose() << endl;

//    	MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
//     x += K*(z - zhat);
//     MatrixXd I = MatrixXd::Zero(16, 16);
//     P = (I - K*H)*P;
//     x.head(4).normalize();
//     this->current_t = t;
// }

//xdot = f(x, u);
VectorXd Pose_ekf::process(Vector3d gyro, Vector3d acc, VectorXd& xdot, MatrixXd& F)
{
	Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Vector3d p = x.segment<3>(4);
	Vector3d v = x.segment<3>(7);
	Vector3d bw = x.segment<3>(10);
	Vector3d ba = x.segment<3>(13);
	xdot = VectorXd::Zero(16);
	F = MatrixXd::Zero(n_state, n_state); 

	Quaterniond gyro_q;
	gyro_q.vec() = gyro - bw;
	Quaterniond q_dot = q*gyro_q;
	q_dot.w() /= 2; q_dot.vec() /= 2;//* and / to scalar is not support for Quaternion
	xdot(0) = q_dot.w();
	xdot.segment<3>(1) = q_dot.vec();
	xdot.segment<3>(4) = v;

	Matrix3d Rnb = quaternion2mat(q);
	Vector3d g = Vector3d(0, 0, 9.8);
	xdot.segment<3>(7) = Rnb*(acc - ba) + g;
	xdot.segment<3>(10) = Vector3d::Zero();
	xdot.segment<3>(13) = Vector3d::Zero();

	F.block<4, 4>(0, 0) = 0.5*diff_pq_p(gyro_q);
	F.block<4, 3>(0, 4) = -0.5*(diff_pq_q(q).block<4, 3>(0, 1));

	F.block<3, 3>(4, 7) = Matrix3d::Identity();
	
	F.block<3, 1>(4, 0) = -diff_qvqstar_q(q, ba);
	F.block<3, 3>(4, 1) = -diff_qvqstar_v(q);
}


// MatrixXd Pose_ekf::computeF(Vector3d gyro, Vector3d acc)
// {
// 	VectorXd delta[16];
// 	for (int i = 0; i < 16; ++i)
// 	{
// 		delta[i] = VectorXd::Zero(16);
// 		delta[i](i) = eps;
// 	}

// 	MatrixXd F = MatrixXd::Zero(16, 16);
// 	for (int i = 0; i < 16; ++i)
// 	{
// 		VectorXd temp = process(x + delta[i], gyro, acc) - process(x - delta[i], gyro, acc);
// 		F.col(i) = temp/(2*eps);
// 	}
// 	return F;
// }


void Pose_ekf::getState(Quaterniond& q, Vector3d& p, Vector3d& v, Vector3d & bw, Vector3d& ba)
{
	q.w() = x(0);
	q.vec() = x.segment<3>(1);
	p = x.segment<3>(4);
	v = x.segment<3>(7);
	bw = x.segment<3>(10);
	ba = x.segment<3>(13);
}


void Pose_ekf::measurement_fix(Vector2d& position, MatrixXd &H)
{
	position = x.segment<2>(4);
	H = MatrixXd::Zero(2, n_state);
	H.block<2, 2>(0, 4) = Matrix2d::Identity();
}
void Pose_ekf::measurement_fix_velocity(Vector3d& velocity, MatrixXd& H)
{
	velocity = x.segment<3>(7);
	H = MatrixXd::Zero(3, n_state);
	H.block<3, 3>(0, 7) = Matrix3d::Identity();
}

void Pose_ekf::measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H)
{
	sonar_height = VectorXd(1);
	sonar_height(0) = x(6);
	H = MatrixXd::Zero(1, n_state);
	H(0, 6) = 1;
}


void Pose_ekf::correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R)
{
    cout << "z: " << z.transpose() << endl;
    cout << "zhat: " << zhat.transpose() << endl;
    cout << "dz: " << (z - zhat).transpose() << endl;

   	MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    x += K*(z - zhat);
    MatrixXd I = MatrixXd::Zero(16, 16);
    P = (I - K*H)*P;
    x.head(4).normalize();
}
void Pose_ekf::correct_fix(Vector3d position, double t)
{
	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);
	double dt = t - current_t;

	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	Vector2d z = position.head(2);
	Vector2d zhat;
	MatrixXd H;
	measurement_fix(zhat, H);
	correct(z, zhat, H, R_fix);
}
void Pose_ekf::correct_fix_velocity(Vector3d velocity, double t)
{
	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);
	double dt = t - current_t;

	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	Vector3d z = velocity;
	Vector3d zhat;
	MatrixXd H;
	measurement_fix_velocity(zhat, H);
	correct(z, zhat, H, R_fix_velocity);
}
void Pose_ekf::correct_sonar_height(double sonar_height, double t)
{
	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);
	double dt = t - current_t;
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	VectorXd z(1);
	z(0) = sonar_height;
	VectorXd zhat(1);
	MatrixXd H;
	//measurement_sonar_height(zhat, H);
	//correct(z, zhat, H, R_fix_velocity);
}