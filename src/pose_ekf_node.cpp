#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "hector_uav_msgs/Altimeter.h"


#include <Eigen/Geometry>
#include <deque>

#include "pose_ekf.h"
#include <algorithm>

using namespace std;
using namespace Eigen;

enum SensorType
{
  IMU,
  FIX,
  FIX_VELOCITY,
  MAGNETIC,
  SONAR,
  ALTIMETER,
};


Pose_ekf pose_ekf;

ros::Publisher pub_path;
ros::Publisher pub_pose;
nav_msgs::Path path_msg;

deque< pair<double, sensor_msgs::Imu> > imu_q;
deque< pair<double, geometry_msgs::Vector3Stamped> > mag_q;
deque< pair<double, hector_uav_msgs::Altimeter> >altimeter_q;
deque< pair<double, sensor_msgs::Range> >sonar_height_q;
deque< pair<double, sensor_msgs::NavSatFix> >fix_q;
deque< pair<double, geometry_msgs::Vector3Stamped> >fix_velocity_q;



//
bool processSensorData()
{
  
  if(imu_q.empty() || (imu_q.back().first - imu_q.front().first) < 0.15 ) return false;

  //find the first com sensor
  double t[6] = {DBL_MAX};
  if(!imu_q.empty()) t[0] = imu_q.front().first;
  if(!mag_q.empty()) t[1] = mag_q.front().first;
  if(!altimeter_q.empty()) t[2] = altimeter_q.front().first;
  if(!sonar_height_q.empty()) t[3] = sonar_height_q.front().first;
  if(!fix_q.empty()) t[4] = fix_q.front().first;
  if(!fix_velocity_q.empty()) t[5] = fix_velocity_q.front().first;

  int min_id = min_element(t, t + 6) - t;
  if(t[min_id] == DBL_MAX) return false;

  switch (min_id)
  {
    case 0:

      break;
    case 1:

      break;
    case 2:

      break;  
    case 3:

      break;
    case 4:

      break;
    case 5:
    
      break;  
  }
  return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    ROS_INFO("imu");
    Vector3d gyro, acc;
    acc(0) = imu_msg->linear_acceleration.x;
    acc(1) = imu_msg->linear_acceleration.y;
    acc(2) = imu_msg->linear_acceleration.z;
    
    gyro(0) = imu_msg->angular_velocity.x;
    gyro(1) = imu_msg->angular_velocity.y;
    gyro(2) = imu_msg->angular_velocity.z;

    double t = imu_msg->header.stamp.toSec();
    imu_q.push_back(make_pair(t, *imu_msg) );
    //pose_ekf.predict(gyro, acc, t);
}

void magCallback(const geometry_msgs::Vector3StampedConstPtr &msg)
{
  Vector3d mag;
  mag(0) = msg->vector.x;
  mag(1) = msg->vector.y;
  mag(2) = msg->vector.z;
  double t = msg->header.stamp.toSec();
  mag_q.push_back(make_pair(t, *msg));
}

void altimeterCallback(const hector_uav_msgs::AltimeterConstPtr& msg)
{
  double t = msg->header.stamp.toSec();
  altimeter_q.push_back(make_pair(t, *msg));
}

void sonarCallback(const sensor_msgs::RangeConstPtr &msg)
{
  double depth  = msg->range / ((msg->range > 100.0) ? 100.0 :1.0);
  double t = msg->header.stamp.toSec();
  sonar_height_q.push_back(make_pair(t, *msg));
}

void fixCallback(const sensor_msgs::NavSatFixConstPtr & msg)
{
  double t = msg->header.stamp.toSec();
  fix_q.push_back(make_pair(t, *msg));
}

void fixVelocityCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
    double t = msg->header.stamp.toSec();
    fix_velocity_q.push_back(make_pair(t, *msg));
}

int main (int argc, char **argv) 
{

  ros::init(argc, argv, "pose_estimator");
  ros::NodeHandle n("~");

  pub_path = n.advertise<nav_msgs::Path>("path", 10);
  pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 10);

  path_msg.header.frame_id = "world";

  ros::Subscriber sub_imu = n.subscribe("imu", 100, imuCallback);
  ros::Subscriber sub_mag = n.subscribe("magnetic_field", 100, magCallback);
  ros::Subscriber sub_fix = n.subscribe("fix", 100, fixCallback);
  ros::Subscriber sub_sonar = n.subscribe("sonar", 100, sonarCallback); 
  ros::Subscriber sub_fix_velocity = n.subscribe("fix_velocity", 100, fixVelocityCallback);
  ros::Subscriber sub_altimeter = n.subscribe("altimeter", 100, altimeterCallback);

  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    ros::spinOnce();

    while(processSensorData()){}
    loop_rate.sleep();
  }
  ros::spin();
}

