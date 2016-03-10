#include "pose_ekf.h"
#include <iostream>
#include <Eigen/Dense>
#include "conversion.h"
using namespace std;
using namespace Eigen;


//quaternion: body fram to navigation frame
//Rnb
//ekf state 
//0-3 quaternion
//4-6 Pn Pe Pd
//7-9 Vn Ve Vd
//10-12 bwx bwy bwz
//13-15 bax bay baz 


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

//diff_(qstar*v*q)/ diff_q
MatrixXd diff_qstarvq_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v - skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() + q0*skew_symmetric(v));
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

Pose_ekf::Pose_ekf()
{	
	initialized = false;
	x = VectorXd::Zero(n_state);
	x.head(4) << 1, 0, 0, 0;
	P = MatrixXd::Identity(n_state, n_state);

	initialized = false;
	fix_initialized = false;
	imu_initialized = false;
	altimeter_initialized = false;
	sonar_initialized = false;
	magnetic_initialized = false;
}

Pose_ekf::~Pose_ekf()
{
	
}


void Pose_ekf::predict(Vector3d gyro, Vector3d acc, double t)
{
	if(!imu_initialized)
	{
		imu_initialized = true;
		this->current_t = t;
		double phy = atan2(acc(1), acc(2));
		double theta = atan2(-acc(0), acc(2));
		Vector3d rpy(phy, theta, 0);
		Quaterniond q = euler2quaternion(rpy);
		x(0) = q.w(); x.segment<3>(1) = q.vec();
		return;
	}

	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	//cout << "line" << __LINE__ << endl;
	double dt = t - current_t;
	cout << "dt: " << dt << endl;
	VectorXd xdot(n_state);
	MatrixXd F(n_state, n_state);
	process(gyro, acc, xdot, F);
	//cout << "line" << __LINE__ << endl;
	x += xdot*dt;
	F = MatrixXd::Identity(n_state, n_state) + F*dt;//continous F and discrete F
	//cout << "line" << __LINE__ << endl;
	
	P = F*P*F.transpose() + Q;//Q and t
	//cout << "F: " << F << endl;
	
	//cout << "line" << __LINE__ << endl;
	//cout << "P: " << P << endl;
	x.head(4).normalize();
	//cout << "line" << __LINE__ << endl;
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
void Pose_ekf::process(Vector3d gyro, Vector3d acc, VectorXd& xdot, MatrixXd& F)
{
	//cout << "line" << __LINE__ << endl;
	//cout << "gyro : " << gyro.transpose() << endl;
	//cout << "acc: " << acc.transpose() << endl;
	Quaterniond q;
	Vector3d p, v, bw, ba;
	getState(q, p, v, bw, ba);
	//cout << "line" << __LINE__ << endl;
	//xdot = VectorXd::Zero(n_state);
	//F = MatrixXd::Zero(n_state, n_state); 
	xdot.setZero();
	F.setZero();
	//cout << "line" << __LINE__ << endl;
	Quaterniond gyro_q(0, 0, 0, 0);
	gyro_q.vec() = gyro - bw;
	Quaterniond q_dot = q*gyro_q;
	q_dot.w() /= 2; q_dot.vec() /= 2;//* and / to scalar is not support for Quaternion
	xdot(0) = q_dot.w();
	xdot.segment<3>(1) = q_dot.vec();
	xdot.segment<3>(4) = v;
	//cout << "line" << __LINE__ << endl;
	//Vector3d g = Vector3d(0, 0, 9.8);
	Quaterniond acc_b_q(0, 0, 0, 0);
	acc_b_q.vec() = acc - ba;
	Quaterniond acc_n_q =  q*acc_b_q*q.inverse();
	xdot.segment<3>(7) = acc_n_q.vec() - GRAVITY;//body frame to n frame 
	//cout << "line" << __LINE__ << endl;
	F.block<4, 4>(0, 0) = 0.5*diff_pq_p(gyro_q);
	F.block<4, 3>(0, 10) = -0.5*(diff_pq_q(q).block<4, 3>(0, 1));

	F.block<3, 3>(4, 7) = Matrix3d::Identity();
	
	F.block<3, 4>(7, 0) = diff_qvqstar_q(q, acc_b_q.vec());
	F.block<3, 3>(7, 13) = -diff_qvqstar_v(q);
	//cout << "line" << __LINE__ << endl;
	//cout << "xdot:" << xdot.transpose() << endl;
	//cout << "F: " << F << endl;
}

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

void Pose_ekf::measurement_magnetic_field(Vector3d& magnetic_field, MatrixXd& H)
{
	Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Quaterniond ref_mag_q;
	ref_mag_q.w() = 0; ref_mag_q.vec() = referenceMagneticField_;
	cout << "ref_mag:" << referenceMagneticField_.transpose() << endl;
	cout << "q: " << q.w()  << " " << q.vec().transpose() << endl;
	Quaterniond magnetic_field_q =  q.inverse()*ref_mag_q*q; //r_n to r_b
	//cout << "magnetic_field_q: " << magnetic_field_q.w()  << " " << magnetic_field_q.vec().transpose() << endl;
	magnetic_field = magnetic_field_q.vec();

	H = MatrixXd::Zero(3, n_state);
	H.block<3, 4>(0, 0) = diff_qstarvq_q(q, referenceMagneticField_);
	cout << "magnetic_field: " << magnetic_field.transpose() << endl;
}

void Pose_ekf::measurement_gravity(Vector3d& acc, MatrixXd& H)
{
	Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Vector3d ba = x.segment<3>(13);
	Quaterniond g_n_q;
	g_n_q.w() = 0; g_n_q.vec() = Vector3d(0, 0, 1);//only direction is used
	Quaterniond acc_q =  q.inverse()*g_n_q*q; //r_n to r_b
	//cout << "magnetic_field_q: " << magnetic_field_q.w()  << " " << magnetic_field_q.vec().transpose() << endl;
	acc = acc_q.vec();

	H = MatrixXd::Zero(3, n_state);
	H.block<3, 4>(0, 0) = diff_qstarvq_q(q, GRAVITY);
	//H.block<3, 3>(0, 13) = Matrix3d::Identity();
	cout << "acc: " << acc.transpose() << endl;
}


void Pose_ekf::correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R)
{
    // cout << "z: " << z.transpose() << endl;
    // cout << "zhat: " << zhat.transpose() << endl;
    // cout << "dz: " << (z - zhat).transpose() << endl;
   	MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
   	//cout << "line" << __LINE__ << endl;
    x += K*(z - zhat);
    //cout << "line" << __LINE__ << endl;
    MatrixXd I = MatrixXd::Identity(n_state, n_state);
    P = (I - K*H)*P;
    x.head(4).normalize();

    //cout << "line" << __LINE__ << endl;

    Quaterniond q;
	Vector3d p, v, bw, ba;
	getState(q, p, v, bw, ba);
	// cout << "q: " << q.w()  << " " << q.vec().transpose() << endl;
	// cout << "p: " << p.transpose() << endl;
	// cout << "v: " << v.transpose() << endl;
	// cout << "bw: " << bw.transpose() << endl;
	// cout << "ba: " << ba.transpose() << endl;
	
}
void Pose_ekf::correct_fix(Vector3d position, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);
	double dt = t - current_t;
	Vector2d z = position.head(2);
	Vector2d zhat;
	MatrixXd H;
	measurement_fix(zhat, H);
	correct(z, zhat, H, R_fix);
}
void Pose_ekf::correct_fix_velocity(Vector3d velocity, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);
	
	Vector3d z = velocity;
	Vector3d zhat;
	MatrixXd H;
	measurement_fix_velocity(zhat, H);
	correct(z, zhat, H, R_fix_velocity);
}
void Pose_ekf::correct_sonar_height(double sonar_height, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);

	VectorXd z(1);
	z(0) = sonar_height;
	VectorXd zhat(1);
	MatrixXd H;

	measurement_sonar_height(zhat, H);
	correct(z, zhat, H, R_sonar_height);
}

void Pose_ekf::correct_magnetic_field(Vector3d mag, double t)
{
	if(!magnetic_initialized)
	{
		//note, mag in ENU should be [0 1 x], but for the simulated data it is [1 0 x], maybe a bug
		referenceMagneticField_(0) = mag.head(2).norm();
		referenceMagneticField_(1) = 0;
		referenceMagneticField_(2) = mag(2);
		magnetic_initialized = true;
		current_t = t;
		return;
	}

	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);
	
	Vector3d z = mag;
	Vector3d zhat;
	MatrixXd H;
	measurement_magnetic_field(zhat, H);
	cout << "mag: " << mag.transpose() << " mag_hat: " << zhat.transpose() << endl;
	correct(z, zhat, H, R_magnetic);
}

void Pose_ekf::correct_gravity(Vector3d acc, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	if(t < current_t) return;
	//predict to current step
	predict(this->gyro, this->acc, t);
	
	Vector3d z = acc/acc.norm();
	Vector3d zhat;
	MatrixXd H;
	measurement_gravity(zhat, H);
	cout << "acc: " << z << " acc_hat: " << zhat.transpose() << endl;
	correct(z, zhat, H, R_gravity);
}