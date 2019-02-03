#ifndef PROJECT_EKF_H
#define PROJECT_EKF_H
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <igvc_msgs/velocity_pair.h>

class Ekf
{
public:
  Ekf();

private:
  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  void imu_callback(const sensor_msgs::ImuConstPtr& imu);
  void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps);
  void odometry_callback(const igvc_msgs::velocity_pairConstPtr& motor);
};

#endif  // PROJECT_EKF_H
