#include "ekf.h"
#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>

Ekf::Ekf() : nh{}, pNh{ "~" }
{
  std::string topic_imu, topic_gps, topic_odometry;

  igvc::getParam(pNh, "topics/imu", topic_imu);
  igvc::getParam(pNh, "topics/gps", topic_gps);
  igvc::getParam(pNh, "topics/odometry", topic_odometry);

  ros::Subscriber imu_sub = nh.subscribe(topic_imu, 1, &Ekf::imu_callback, this);
  ros::Subscriber gps_sub = nh.subscribe(topic_gps, 1, &Ekf::gps_callback, this);
  ros::Subscriber odom_sub = nh.subscribe(topic_odometry, 1, &Ekf::odometry_callback, this);
}

void Ekf::imu_callback(const sensor_msgs::ImuConstPtr &imu) {

}
void Ekf::gps_callback(const sensor_msgs::NavSatFixConstPtr &gps) {

}
void Ekf::odometry_callback(const igvc_msgs::velocity_pairConstPtr &motor) {

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "StateEstimator");
  Ekf ekf;
}
