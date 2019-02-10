#ifndef PROJECT_EKF_H
#define PROJECT_EKF_H
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <igvc_msgs/velocity_pair.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <mutex>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class Ekf
{
public:
  Ekf();

private:
  bool m_initialized;
  int m_buffer_size;
  int m_state_dims;

  double m_kappa;
  double m_alpha;
  double m_beta;
  double m_gamma;
  double m_lambda;
  double m_phi1, m_phi2;
  double m_axle_length;

  std::string m_odom_frame;
  std::string m_base_frame;

  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  ros::Publisher m_pose_pub;

  std::vector<double> m_initial_pose;

  Vector6d m_mu_predicted;
  Vector6d m_state;
  Eigen::Matrix<double, 13, 1>  m_weights_m;
  Eigen::Matrix<double, 13, 1> m_weights_c;
  Matrix6d m_covariance;
  Matrix6d m_noise_predict;
  Matrix6d m_covar_predicted;

  Eigen::Matrix<double, 2, 2> m_odom_noise;
  Eigen::Matrix<double, 4, 4> m_imu_noise;
  Eigen::Matrix<double, 3, 3> m_gps_noise;

  enum Sensor { none, imu, gps, odom };

  boost::circular_buffer<sensor_msgs::ImuConstPtr> m_imu_buffer;
  boost::circular_buffer<sensor_msgs::NavSatFixConstPtr> m_gps_buffer;
  boost::circular_buffer<igvc_msgs::velocity_pairConstPtr> m_odom_buffer;

  std::mutex m_imu_buffer_mutex;
  std::mutex m_gps_buffer_mutex;
  std::mutex m_odom_buffer_mutex;

  ros::Time m_last_update_time;

  void imu_callback(const sensor_msgs::ImuConstPtr& imu);
  void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps);
  void odometry_callback(const igvc_msgs::velocity_pairConstPtr& motor);

  void imu_update();
  void gps_update();
  void odom_update();

  void sigma_to_odom(const Eigen::Matrix<double, 6, 13>& sigma_points, Eigen::Matrix<double, 2, 13>& Z_predicted);
  void sigma_to_imu(const Eigen::Matrix<double, 6, 13>& sigma_points, Eigen::Matrix<double, 4, 13>& Z_predicted);
  void sigma_to_gps(const Eigen::Matrix<double, 6, 13>& sigma_points, Eigen::Matrix<double, 3, 13>& Z_predicted);

  void prediction_step(const ros::Time& target_time);
  void motion_model(const ros::Duration &update_duration, const Eigen::Matrix<double, 6, 13> &sigma,
                         Eigen::Matrix<double, 6, 13>& sigma_star);
  void recalculate_sigma_points(Eigen::Matrix<double, 6, 13>& sigma_points);

  void compute_gps_diff(const sensor_msgs::NavSatFixConstPtr& gps);
  void publish_pose() const;
  void calculate_process_noise(double dt);
  double gps_x;
  double gps_y;
  double gps_theta;

#define STAMP(x) (x->header.stamp)

  Sensor next_sensor();

  void main_loop();
};

#endif  // PROJECT_EKF_H
