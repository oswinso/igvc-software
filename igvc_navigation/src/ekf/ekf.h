#ifndef PROJECT_EKF_H
#define PROJECT_EKF_H
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <igvc_msgs/velocity_pair.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class Ekf
{
public:
  Ekf();

private:
  int m_buffer_size;
  int m_state_dims;

  double m_gamma;

  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  std::vector<double> m_initial_pose;

  Vector6d m_state;
  Vector6d m_weights_m;
  Vector6d m_weights_c;
  Matrix6d m_covariance;
  Matrix6d m_noise_predict;
  Matrix6d m_mu_predicted;
  Matrix6d m_covar_predicted;

  enum Sensor { none, imu, gps, odom };

  boost::circular_buffer<sensor_msgs::ImuConstPtr> m_imu_buffer;
  boost::circular_buffer<sensor_msgs::NavSatFixConstPtr> m_gps_buffer;
  boost::circular_buffer<igvc_msgs::velocity_pairConstPtr> m_odom_buffer;

  ros::Time m_last_update_time;

  void imu_callback(const sensor_msgs::ImuConstPtr& imu);
  void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps);
  void odometry_callback(const igvc_msgs::velocity_pairConstPtr& motor);

  void imu_update();
  void gps_update();
  void odom_update();

  void sigma_to_odom(Eigen::Matrix<double, 6, 13> sigma_points, Eigen::Matrix<double, 2, 13> Z_predicted);
  void sigma_to_imu(Eigen::Matrix<double, 6, 13> sigma_points, Eigen::Matrix<double, 4, 13> Z_predicted);
  void sigma_to_gps(Eigen::Matrix<double, 6, 13> sigma_points, Eigen::Matrix<double, 3, 13> Z_predicted);

  void prediction_step(ros::Time target_time);
  void motion_model(const ros::Duration &update_duration, const Eigen::Matrix<double, 6, 13> &sigma,
                         Eigen::Matrix<double, 6, 13>& sigma_star);
  void recalculate_sigma_points(Eigen::Matrix<double, 6, 13>& sigma_points);

#define STAMP(x) (x->header.stamp)

  Sensor next_sensor();

  void main_loop();
};

#endif  // PROJECT_EKF_H
