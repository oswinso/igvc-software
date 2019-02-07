#include "ekf.h"
#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>
#include <thread>

#define x(mat) (mat.row(0))
#define y(mat) (mat.row(1))
#define v(mat) (mat.row(2))
#define a(mat) (mat.row(3))
#define theta(mat) (mat.row(4))
#define theta_dot(mat) (mat.row(5))

Ekf::Ekf() : m_state_dims{6}, nh{}, pNh{"~"}, m_state{}
{
  std::string topic_imu, topic_gps, topic_odometry;

  igvc::getParam(pNh, "topics/imu", topic_imu);
  igvc::getParam(pNh, "topics/gps", topic_gps);
  igvc::getParam(pNh, "topics/odometry", topic_odometry);
  igvc::getParam(pNh, "ekf/initial_pose", m_initial_pose);
  igvc::getParam(pNh, "other/buffer_size", m_buffer_size);
  igvc::getParam(pNh, "motion_model/axle_length", m_axle_length);

  ros::Subscriber imu_sub = nh.subscribe(topic_imu, 1, &Ekf::imu_callback, this);
  ros::Subscriber gps_sub = nh.subscribe(topic_gps, 1, &Ekf::gps_callback, this);
  ros::Subscriber odom_sub = nh.subscribe(topic_odometry, 1, &Ekf::odometry_callback, this);

  m_imu_buffer = boost::circular_buffer<sensor_msgs::ImuConstPtr>(m_buffer_size);
  m_gps_buffer = boost::circular_buffer<sensor_msgs::NavSatFixConstPtr>(m_buffer_size);
  m_odom_buffer = boost::circular_buffer<igvc_msgs::velocity_pairConstPtr>(m_buffer_size);

  std::thread main_thread(&Ekf::main_loop, this);
  ros::spin();
}

void Ekf::imu_callback(const sensor_msgs::ImuConstPtr &imu)
{
  m_imu_buffer.push_back(imu);
}

void Ekf::gps_callback(const sensor_msgs::NavSatFixConstPtr &gps)
{
  m_gps_buffer.push_back(gps);
}

void Ekf::odometry_callback(const igvc_msgs::velocity_pairConstPtr &odom)
{
  m_odom_buffer.push_back(odom);
}

void Ekf::main_loop()
{
  // Setup EKF

  // Initialize state:
  // x, y, v, a, theta, theta'
  m_state(0) = m_initial_pose[0];
  m_state(1) = m_initial_pose[1];
  m_state(2) = m_initial_pose[2];
  m_state(3) = m_initial_pose[3];
  m_state(4) = m_initial_pose[4];
  m_state(5) = m_initial_pose[5];


  while (ros::ok()) {
    // Get message from buffer with earliest timestamp
    Sensor sensor = next_sensor();
    switch (sensor) {
      case Sensor::odom:
        odom_update();
        break;
      case Sensor::imu:
        imu_update();
      case Sensor::gps:
        gps_update();
      case Sensor::none:
      default:
        return;
    }
  }
}

void Ekf::odom_update()
{
  // TODO: populate sensor
  Eigen::Matrix<double, 3, 1> sensor;

  // 1. Predict till the message
  prediction_step(STAMP(m_odom_buffer.front()));
  Eigen::Matrix<double, 6, 13> sigma_points;

  // 2. Recalculate sigma points. χ = (μ    μ+γ*sqrt(Σ)     μ-γ*sqrt(Σ))
  recalculate_sigma_points(sigma_points);
  Eigen::Matrix<double, 3, 13> Z_predicted;

  // 3. Transform sigma points from state space to measurement space. Z = h(χ)
  sigma_to_odom(sigma_points, Z_predicted);

  // 4. Compute predicted observation. z_predicted = Zw
  Eigen::Matrix<double, 3, 1> z_predicted = Z_predicted * m_weights_m;

  // 5. Compute uncertainty in transform + observation uncertainty. S = (Z - z) * w * (Z - z)^T + Q
  Eigen::Matrix<double, 3, 13> Z_diff;
  for (int i = 0; i < 13; i++)
  {
    Z_diff.col(i) = Z_predicted.col(i) - z_predicted;
  }
  Eigen::Matrix<double, 3, 3> S = Z_diff * m_weights_c * Z_diff.transpose() + m_noise_measure;


  // 6. Compute cross co-relation matrix between state and measurement space. T = (χ - μ) (Z - z)^T * w
  Eigen::Matrix<double, 6, 13> sigma_diff;
  for (int i = 0; i < 6; i++)
  {
    sigma_points.col(i) - m_mu_predicted;
  }
  Eigen::Matrix<double, 6, 3> cross_corelation = sigma_diff * m_weights_c * Z_diff.transpose();

  // 7. Compute Kalman Gain. K = T S^-1
  Eigen::Matrix<double, 3, 3> kalman_gain = cross_corelation * S.inverse();

  // 8. Compute corrected mean. μ_corrected = μ_predicted + K(z_observed - z_predicted)
  m_state = m_mu_predicted + kalman_gain * (sensor - z_predicted);

  // 9. Compute corrected covariance. Sigma = Simga_predicted - K S K^T
  m_covariance = m_covar_predicted - kalman_gain * S * kalman_gain.transpose();
}

void Ekf::sigma_to_odom(Eigen::Matrix<double, 6, 13> sigma_points, Eigen::Matrix<double, 2, 13> Z_pred)
{
  // Convert from:
  // x, y, v, a, theta, theta'
  // To:
  // V_l, V_r
  //
  // V_l = (2v - theta')/2
  // V_r = (2v + theta')/2
#define vl(mat) (mat.row(0))
#define vr(mat) (mat.row(1))
  vl(Z_pred) = (2 * v(sigma_points) - m_axle_length * theta_dot(sigma_points))/2;
  vr(Z_pred) = (2 * v(sigma_points) + m_axle_length * theta_dot(sigma_points))/2;
}

void Ekf::imu_update()
{
  // TODO: populate sensor
  Eigen::Matrix<double, 4, 1> sensor;

  // 1. Predict till the message
  prediction_step(STAMP(m_imu_buffer.front()));
  Eigen::Matrix<double, 6, 13> sigma_points;

  // 2. Recalculate sigma points. χ = (μ    μ+γ*sqrt(Σ)     μ-γ*sqrt(Σ))
  recalculate_sigma_points(sigma_points);
  Eigen::Matrix<double, 4, 13> Z_predicted;

  // 3. Transform sigma points from state space to measurement space. Z = h(χ)
  sigma_to_imu(sigma_points, Z_predicted);

  // 4. Compute predicted observation. z_predicted = Zw
  Eigen::Matrix<double, 4, 1> z_predicted = Z_predicted * m_weights_m;

  // 5. Compute uncertainty in transform + observation uncertainty. S = (Z - z) * w * (Z - z)^T + Q
  Eigen::Matrix<double, 4, 13> Z_diff;
  for (int i = 0; i < 13; i++)
  {
    Z_diff.col(i) = Z_predicted.col(i) - z_predicted;
  }
  Eigen::Matrix<double, 4, 4> S = Z_diff * m_weights_c * Z_diff.transpose() + m_noise_measure;


  // 6. Compute cross co-relation matrix between state and measurement space. T = (χ - μ) (Z - z)^T * w
  Eigen::Matrix<double, 6, 13> sigma_diff;
  for (int i = 0; i < 6; i++)
  {
    sigma_points.col(i) - m_mu_predicted;
  }
  Eigen::Matrix<double, 6, 4> cross_corelation = sigma_diff * m_weights_c * Z_diff.transpose();

  // 7. Compute Kalman Gain. K = T S^-1
  Eigen::Matrix<double, 4, 4> kalman_gain = cross_corelation * S.inverse();

  // 8. Compute corrected mean. μ_corrected = μ_predicted + K(z_observed - z_predicted)
  m_state = m_mu_predicted + kalman_gain * (sensor - z_predicted);

  // 9. Compute corrected covariance. Sigma = Simga_predicted - K S K^T
  m_covariance = m_covar_predicted - kalman_gain * S * kalman_gain.transpose();
}

void Ekf::sigma_to_imu(Eigen::Matrix<double, 6, 13> sigma_points, Eigen::Matrix<double, 4, 13> Z_pred)
{
  // Convert from:
  // x, y, v, a, theta, theta'
  // To:
  // x'', y'', theta, theta'
  //
  // x'' = a * cos(theta)
  // y'' = a * sin(theta)
  // theta = theta
  // theta' = theta'
#define acc_x(mat) (mat.row(0))
#define acc_y(mat) (mat.row(1))
#define theta_z(mat) (mat.row(2))
#define theta_dot_z(mat) (mat.row(3))

  acc_x(Z_pred) = (a(sigma_points).array() * theta(sigma_points).array().cos()).matrix();
  acc_y(Z_pred) = (a(sigma_points).array() * theta(sigma_points).array().sin()).matrix();
  theta_z(Z_pred) = theta(sigma_points);
  theta_dot_z(Z_pred) = theta_dot(sigma_points);
}

void Ekf::gps_update()
{
  // TODO: populate sensor
  sensor_msgs::NavSatFixConstPtr gps_data = m_gps_buffer.front();
  compute_gps_diff(gps_data);
  Eigen::Matrix<double, 3, 1> sensor;
  // 1. Predict till the message
  prediction_step(STAMP(gps_data));
  Eigen::Matrix<double, 6, 13> sigma_points;

  // 2. Recalculate sigma points. χ = (μ    μ+γ*sqrt(Σ)     μ-γ*sqrt(Σ))
  recalculate_sigma_points(sigma_points);

  // 3. Transform sigma points from state space to measurement space. Z = h(χ)
  Eigen::Matrix<double, 3, 13> Z_predicted;
  sigma_to_imu(sigma_points, Z_predicted);

  // 4. Compute predicted observation. z_predicted = Zw
  Eigen::Matrix<double, 3, 1> z_predicted = Z_predicted * m_weights_m;

  // 5. Compute uncertainty in transform + observation uncertainty. S = (Z - z) * w * (Z - z)^T + Q
  Eigen::Matrix<double, 3, 13> Z_diff;
  for (int i = 0; i < 13; i++)
  {
    Z_diff.col(i) = Z_predicted.col(i) - z_predicted;
  }
  Eigen::Matrix<double, 3, 3> S = Z_diff * m_weights_c * Z_diff.transpose() + m_noise_measure;


  // 6. Compute cross co-relation matrix between state and measurement space. T = (χ - μ) (Z - z)^T * w
  Eigen::Matrix<double, 6, 13> sigma_diff;
  for (int i = 0; i < 6; i++)
  {
    sigma_points.col(i) - m_mu_predicted;
  }
  Eigen::Matrix<double, 6, 3> cross_corelation = sigma_diff * m_weights_c * Z_diff.transpose();

  // 7. Compute Kalman Gain. K = T S^-1
  Eigen::Matrix<double, 3, 3> kalman_gain = cross_corelation * S.inverse();

  // 8. Compute corrected mean. μ_corrected = μ_predicted + K(z_observed - z_predicted)
  m_state = m_mu_predicted + kalman_gain * (sensor - z_predicted);

  // 9. Compute corrected covariance. Sigma = Simga_predicted - K S K^T
  m_covariance = m_covar_predicted - kalman_gain * S * kalman_gain.transpose();
}

void Ekf::sigma_to_gps(Eigen::Matrix<double, 6, 13> sigma_points, Eigen::Matrix<double, 3, 13> Z_pred)
{
  // Convert from:
  // x, y, v, a, theta, theta'
  // To:
  // x, y, theta
  //
  // x =
  // y =
  // theta =
  x(Z_pred) = x(sigma_points);
  y(Z_pred) = y(sigma_points);
  theta_z(Z_pred) = theta(sigma_points);
}

/**
 * Converts longitude and latitude into /odom x, y coordinates, calculates theta' from difference
 * @param gps
 */
void compute_gps_diff(const sensor_msgs::NavSatFixConstPtr& gps)
{

}

inline void Ekf::recalculate_sigma_points(Eigen::Matrix<double, 6, 13>& sigma)
{
  // Calculate offset = gamma * sqrt(covariance) through Cholesky decomposition
  Matrix6d offset{m_covar_predicted.ldlt().matrixL()};
  offset = m_gamma * offset;

  // First col is average
  sigma.col(0) = m_mu_predicted;
  // Col 1 -> n is average + gamma * covariance
  sigma.block<6, 6>(1, 0) = m_mu_predicted + offset;
  // Col n+1 -> 2n + 1 is average - gamma * covariance
  sigma.block<6, 6>(1, 0) = m_mu_predicted - offset;
}

/**
 * Predicts state at target_time using differential drive motion model
 * @param target_time Time to predict till
 */
void Ekf::prediction_step(ros::Time target_time)
{
  static Eigen::Array<double, 6, 6> weight_arr = (m_weights_c * m_weights_c.transpose()).array();
  // Calculate how much time we need to predict
  ros::Duration dt = target_time - m_last_update_time;

  // Calculate offset = gamma * sqrt(covariance) through Cholesky decomposition
  Matrix6d offset{m_covariance.ldlt().matrixL()};
  offset = m_gamma * offset;

  // Calculate Sigma points
  Eigen::Matrix<double, 6, 13> sigma;
  // First col is average
  sigma.col(0) = m_state;
  // Col 1 -> n is average + gamma * covariance
  sigma.block<6, 6>(1, 0) = m_state + offset;
  // Col n+1 -> 2n + 1 is average - gamma * covariance
  sigma.block<6, 6>(1, 0) = m_state - offset;

  // Predict until target_time using motion model
  Eigen::Matrix<double, 6, 13> sigma_star;
  motion_model(dt, sigma, sigma_star);

  // Calculate predicted mean
  m_mu_predicted = sigma_star * m_weights_m;

  // Calculate predicted covariance
  m_covar_predicted =
      (((sigma_star - m_mu_predicted) * (sigma_star - m_mu_predicted).transpose()).array() * weight_arr).matrix() +
      m_noise_predict;
}

void Ekf::motion_model(const ros::Duration &dt, const Eigen::Matrix<double, 6, 13> &sigma,
                       Eigen::Matrix<double, 6, 13> &sigma_star)
{
  // Calculate in rows?
  // x, y, v, a, theta, theta'
  //
  // x = x + (v + 0.5 * a * dt^2) * cos(theta + dt * theta / 2) * dt
  // y = y + v * sin ( theta + dt * theta' / 2) * dt + 0.5 * a * dt^2 * sin(theta + dt * theta / 2)
  // v = v + dt * a (But we use v = v + dt * a /2 for x, y calculation)
  // a = a
  // theta = theta + dt * theta' (But we use theta = theta + dt * theta / 2)
  // theta' = theta'

  Eigen::Matrix<double, 1, 13> mid_theta = (theta(sigma) + dt.toSec() / 2 * theta_dot(sigma));
  theta(sigma_star) = theta(sigma) + dt.toSec() * theta(sigma);
  theta_dot(sigma_star) = theta_dot(sigma_star);

  a(sigma_star) = a(sigma);
  v(sigma_star) = v(sigma) + dt.toSec() * a(sigma);
  Eigen::Matrix<double, 1, 13> dpos = (v(sigma) + 0.5 * dt.toSec() * a(sigma));

  x(sigma_star) = x(sigma) + dpos * mid_theta.array().cos().matrix() * dt.toSec();
  y(sigma_star) = y(sigma) + dpos * mid_theta.array().sin().matrix() * dt.toSec();
}


Ekf::Sensor Ekf::next_sensor()
{
  if (m_gps_buffer.empty() && m_imu_buffer.empty() && m_odom_buffer.empty()) {
    return Sensor::none;
  }
  if (STAMP(m_odom_buffer.front()) < STAMP(m_imu_buffer.front())
      && STAMP(m_odom_buffer.front()) < STAMP(m_gps_buffer.front())) {
    return Sensor::odom;
  } else if (STAMP(m_imu_buffer.front()) < STAMP(m_odom_buffer.front())
             && STAMP(m_imu_buffer.front()) < STAMP(m_gps_buffer.front())) {
    return Sensor::imu;
  } else {
    return Sensor::gps;
  }
}