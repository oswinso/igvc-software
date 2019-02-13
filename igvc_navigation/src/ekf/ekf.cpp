#include "ekf.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <igvc_utils/NodeUtils.hpp>
#include <thread>

#define x(mat) (mat.row(0))
#define y(mat) (mat.row(1))
#define v(mat) (mat.row(2))
#define a(mat) (mat.row(3))
#define theta(mat) (mat.row(4))
#define theta_dot(mat) (mat.row(5))

Ekf::Ekf() : m_initialized{ false }, m_state_dims{ 6 }, nh{}, pNh{ "~" }, m_state{}
{
  std::string topic_imu, topic_gps, topic_odometry;
  std::vector<double> covariance_diagonal;
  std::vector<double> imu_covar;
  std::vector<double> odom_covar;
  std::vector<double> gps_covar;

  igvc::getParam(pNh, "topics/imu", topic_imu);
  igvc::getParam(pNh, "topics/gps", topic_gps);
  igvc::getParam(pNh, "topics/odometry", topic_odometry);
  igvc::getParam(pNh, "tf_frames/odom", m_odom_frame);
  igvc::getParam(pNh, "tf_frames/base", m_base_frame);
  igvc::getParam(pNh, "ukf/initial_pose", m_initial_pose);
  igvc::getParam(pNh, "ukf/initial_covariance_diagonal", covariance_diagonal);
  igvc::getParam(pNh, "ukf/kappa", m_kappa);
  igvc::getParam(pNh, "ukf/alpha", m_alpha);
  igvc::getParam(pNh, "ukf/beta", m_beta);
  igvc::getParam(pNh, "ukf/process_noise/phi_acceleration", m_phi1);
  igvc::getParam(pNh, "ukf/process_noise/phi_angular_velocity", m_phi2);
  igvc::getParam(pNh, "other/buffer_size", m_buffer_size);
  igvc::getParam(pNh, "motion_model/axle_length", m_axle_length);
  igvc::getParam(pNh, "imu/covariance", imu_covar);
  igvc::getParam(pNh, "odom/covariance", odom_covar);
  igvc::getParam(pNh, "gps/covariance", gps_covar);

  ros::Subscriber imu_sub = nh.subscribe(topic_imu, 1, &Ekf::imu_callback, this);
  ros::Subscriber gps_sub = nh.subscribe(topic_gps, 1, &Ekf::gps_callback, this);
  ros::Subscriber odom_sub = nh.subscribe(topic_odometry, 1, &Ekf::odometry_callback, this);

  m_imu_buffer = boost::circular_buffer<sensor_msgs::ImuConstPtr>(m_buffer_size);
  m_gps_buffer = boost::circular_buffer<sensor_msgs::NavSatFixConstPtr>(m_buffer_size);
  m_odom_buffer = boost::circular_buffer<igvc_msgs::velocity_pairConstPtr>(m_buffer_size);
  m_covariance = Vector6d(covariance_diagonal.data()).asDiagonal();

  m_pose_pub = nh.advertise<nav_msgs::Odometry>("/odom/ukf", 1);

  // Calculate parameters and weights
  m_state_dims = 6;
  m_gamma = sqrt(m_alpha * m_alpha * (m_state_dims + m_kappa));
  m_lambda = m_alpha * m_alpha * (m_state_dims + m_kappa) - m_state_dims;
  m_weights_m(0) = m_lambda / (m_state_dims + m_lambda);
  m_weights_c(0) = m_weights_m(0) + (1 - m_alpha * m_alpha + m_beta);
  for (int i = 1; i < 13; i++)
  {
    m_weights_m(i) = 1.0 / (2.0 * (m_state_dims + m_lambda));
    m_weights_c(i) = m_weights_m(i);
  }
  ROS_INFO_STREAM("UKF Parameters:\n alpha: " << m_alpha << ", beta: " << m_beta << ", kappa: " << m_kappa
                                              << ", lambda: " << m_lambda << "\nWeights_m: \n"
                                              << m_weights_m << "\nWeights_c: \n"
                                              << m_weights_c);
  double sum_weights = 0;
  for (int i = 0; i < 13; i++)
  {
    sum_weights += m_weights_m(i);
  }
  ROS_INFO_STREAM("Sum of m_weights_m: " << sum_weights);

  // Check for right dimensions
  if (odom_covar.size() != 4 || gps_covar.size() != 9 || imu_covar.size() != 16)
  {
    ROS_ERROR("Covariance matrices have the wrong dimensions...Exiting");
    ros::shutdown();
  }
  // Convert from std::vector to Eigen matrix for noise
  m_odom_noise(0, 0) = odom_covar[0];
  m_odom_noise(0, 1) = odom_covar[1];
  m_odom_noise(1, 0) = odom_covar[2];
  m_odom_noise(1, 1) = odom_covar[3];

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      m_gps_noise(i, j) = gps_covar[3 * i + j];
    }
  }

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      m_imu_noise(i, j) = imu_covar[4 * i + j];
    }
  }

  std::thread main_thread(&Ekf::main_loop, this);
  ros::spin();
}

void Ekf::imu_callback(const sensor_msgs::ImuConstPtr &imu)
{
  std::lock_guard<std::mutex> imu_guard(m_imu_buffer_mutex);
  m_imu_buffer.push_back(imu);
}

void Ekf::gps_callback(const sensor_msgs::NavSatFixConstPtr &gps)
{
  std::lock_guard<std::mutex> gps_guard(m_gps_buffer_mutex);
  m_gps_buffer.push_back(gps);
}

void Ekf::odometry_callback(const igvc_msgs::velocity_pairConstPtr &odom)
{
  std::lock_guard<std::mutex> odom_guard(m_odom_buffer_mutex);
  m_odom_buffer.push_back(odom);
}

void Ekf::main_loop()
{
  ROS_INFO_STREAM("Main thread started");

  // Initialize state:
  // x, y, v, a, theta, theta'
  m_state(0) = m_initial_pose[0];
  m_state(1) = m_initial_pose[1];
  m_state(2) = m_initial_pose[2];
  m_state(3) = m_initial_pose[3];
  m_state(4) = m_initial_pose[4];
  m_state(5) = m_initial_pose[5];

  while (ros::ok())
  {
    // Get message from buffer with earliest timestamp
    Sensor sensor = next_sensor();
    switch (sensor)
    {
      case Sensor::odom:
        odom_update();
        break;
      case Sensor::imu:
        imu_update();
        break;
      case Sensor::gps:
        gps_update();
        break;
      case Sensor::none:
      default:
        continue;
    }
  }
}

void Ekf::odom_update()
{
  if (!m_initialized)
  {
    m_last_update_time = m_odom_buffer.front()->header.stamp;
    m_initialized = true;
    std::lock_guard<std::mutex> odom_guard(m_odom_buffer_mutex);
    m_odom_buffer.pop_front();
    return;
  }
  ROS_INFO_STREAM(std::setw(20) << "\n\n\n========================== ODOM =================================");
  // Populate sensor matrix
  Eigen::Matrix<double, 2, 1> sensor;
  sensor(0) = m_odom_buffer.front()->left_velocity;
  sensor(1) = m_odom_buffer.front()->right_velocity;

  // 1. Predict till the message
  prediction_step(STAMP(m_odom_buffer.front()));

  // 2. Recalculate sigma points. χ = (μ    μ+γ*sqrt(Σ)     μ-γ*sqrt(Σ))
  Eigen::Matrix<double, 6, 13> sigma_points;
  recalculate_sigma_points(sigma_points);

  // 3. Transform sigma points from state space to measurement space. Z = h(χ)
  Eigen::Matrix<double, 2, 13> Z_predicted;
  sigma_to_odom(sigma_points, Z_predicted);
  //  ROS_INFO_STREAM("Predicted Z: \n" << std::setprecision(2) << Z_predicted);

  // 4. Compute predicted observation. z_predicted = Zw
  Eigen::Matrix<double, 2, 1> z_predicted = Eigen::Matrix<double, 2, 1>::Zero();
  for (int i = 1; i < 7; i++)
  {
    z_predicted += Z_predicted.col(i) * m_weights_m(i) + Z_predicted.col(i + 6) * m_weights_m(i + 6);
  }
  z_predicted += Z_predicted.col(0) * m_weights_m(0);
  //  ROS_INFO_STREAM("Predicted z from combining weights of Z_predicted: \n" << std::setprecision(2) << z_predicted);

  // 5. Compute uncertainty in transform + observation uncertainty. S = (Z - z) * w * (Z - z)^T + Q
  Eigen::Matrix<double, 2, 13> Z_diff;
  for (int i = 0; i < 13; i++)
  {
    Eigen::Matrix<double, 3, 3> m_imu_nose;
    Z_diff.col(i) = Z_predicted.col(i) - z_predicted;
  }
  ROS_INFO_STREAM("Difference of each column of Z and z_predicted: \n" << std::setprecision(2) << Z_diff);
  Eigen::Matrix<double, 13, 13> weight_c_matrix = m_weights_c.asDiagonal();
  //  ROS_INFO_STREAM("Weight matrix m_weights_c: \n" << std::setprecision(2) << weight_c_matrix);
  //  ROS_INFO_STREAM("Odometry noise: \n" << std::setprecision(2) << m_odom_noise);
  Eigen::Matrix<double, 2, 2> S = Z_diff * weight_c_matrix * Z_diff.transpose() + m_odom_noise;

  //  ROS_INFO_STREAM("S: \n" << std::setprecision(2) << S);

  // 6. Compute cross co-relation matrix between state and measurement space. T = (χ - μ) (Z - z)^T * w
  Eigen::Matrix<double, 6, 13> sigma_diff;
  for (int i = 0; i < 13; i++)
  {
    sigma_diff.col(i) = sigma_points.col(i) - m_mu_predicted;
  }
  //  ROS_INFO_STREAM("Sigma points after recalculation: \n" << std::setprecision(2) << sigma_points);
  //  ROS_INFO_STREAM("m_mu_predicted: \n" << std::setprecision(2) << m_mu_predicted);
  ROS_INFO_STREAM("Difference between each sigma point and m_mu_predicted: \n" << std::setprecision(2) <<
  sigma_diff);
//  Eigen::Matrix<double, 6, 2> cross_correlation = sigma_diff * weight_c_matrix * Z_diff.transpose();
  Eigen::Matrix<double, 6, 2> cross_correlation = Eigen::Matrix<double, 6, 2>::Zero();
  for (int i = 1; i < 6; ++i)
  {
    cross_correlation += m_weights_c(i) * sigma_diff.col(i) * Z_diff.col(i).transpose() +
                         m_weights_c(i) * sigma_diff.col(i + 6) * Z_diff.col(i + 6).transpose();
  }
  cross_correlation += m_weights_c(0) * sigma_diff.col(0) * Z_diff.col(0).transpose();

  ROS_INFO_STREAM("Cross Correlation: \n" << std::setprecision(2) << cross_correlation);
  ROS_INFO_STREAM("S: \n" << std::setprecision(2) << S);
  ROS_INFO_STREAM("S inverse: \n" << std::setprecision(2) << S.inverse());

  // 7. Compute Kalman Gain. K = T S^-1
  Eigen::Matrix<double, 6, 2> kalman_gain = cross_correlation * S.inverse();

  //  ROS_INFO_STREAM("m_mu_predicted: \n" << std::setprecision(2) << m_mu_predicted);
  ROS_INFO_STREAM("====== Kalman gain: ==========\n" << std::setprecision(2) << kalman_gain);
  ROS_INFO_STREAM("\n========== Sensor ==========\n" << std::setprecision(2) << sensor);
  ROS_INFO_STREAM("========== z_predicted ==========\n" << std::setprecision(2) << z_predicted);
  ROS_INFO_STREAM("========== Sensor - z_predicted ===========\n" << std::setprecision(2) << (sensor - z_predicted));
  ROS_INFO_STREAM("========== Kalman * diff ===========\n"
                  << std::setprecision(2) << kalman_gain * (sensor - z_predicted));
  // 8. Compute corrected mean. μ_corrected = μ_predicted + K(z_observed - z_predicted)
  m_state = m_mu_predicted + kalman_gain * (sensor - z_predicted);

  ROS_INFO_STREAM("\n\n==========Final state===========\n" << std::setprecision(2) << m_state);

  // 9. Compute corrected covariance. Sigma = Simga_predicted - K S K^T
  m_covariance = m_covar_predicted - kalman_gain * S * kalman_gain.transpose();
  ROS_INFO_STREAM("Covariance: \n" << std::setprecision(2) << m_covariance);
  m_last_update_time = m_odom_buffer.front()->header.stamp;

  // Publish pose
  publish_pose();
  std::lock_guard<std::mutex> odom_guard(m_odom_buffer_mutex);
  m_odom_buffer.pop_front();
}

void Ekf::sigma_to_odom(const Eigen::Matrix<double, 6, 13> &sigma_points, Eigen::Matrix<double, 2, 13> &Z_pred)
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
  vl(Z_pred) = (2 * v(sigma_points) - m_axle_length * theta_dot(sigma_points)) / 2;
  vr(Z_pred) = (2 * v(sigma_points) + m_axle_length * theta_dot(sigma_points)) / 2;

  ROS_INFO_STREAM("Transforming sigma points to odom measurements:\n" << std::setprecision(2) << Z_pred);
}

inline double get_yaw(const geometry_msgs::Quaternion &quat)
{
  tf::Matrix3x3 mat(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  double r, p, y;
  mat.getRPY(r, p, y);
  return y;
}

void Ekf::imu_update()
{
  // TODO: populate sensor
  Eigen::Matrix<double, 4, 1> sensor;
  sensor(0) = m_imu_buffer.front()->linear_acceleration.x;
  sensor(1) = m_imu_buffer.front()->linear_acceleration.y;
  sensor(2) = get_yaw(m_imu_buffer.front()->orientation);
  sensor(3) = m_imu_buffer.front()->angular_velocity.z;

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
  Eigen::Matrix<double, 4, 4> S = Z_diff * m_weights_c.asDiagonal() * Z_diff.transpose() + m_imu_noise;

  // 6. Compute cross co-relation matrix between state and measurement space. T = (χ - μ) (Z - z)^T * w
  Eigen::Matrix<double, 6, 13> sigma_diff;
  for (int i = 0; i < 6; i++)
  {
    sigma_points.col(i) - m_mu_predicted;
  }
  Eigen::Matrix<double, 6, 4> cross_corelation = sigma_diff * m_weights_c.asDiagonal() * Z_diff.transpose();

  // 7. Compute Kalman Gain. K = T S^-1
  Eigen::Matrix<double, 6, 4> kalman_gain = cross_corelation * S.inverse();

  // 8. Compute corrected mean. μ_corrected = μ_predicted + K(z_observed - z_predicted)
  m_state = m_mu_predicted + kalman_gain * (sensor - z_predicted);

  // 9. Compute corrected covariance. Sigma = Simga_predicted - K S K^T
  m_covariance = m_covar_predicted - kalman_gain * S * kalman_gain.transpose();

  // Publish pose
  publish_pose();

  m_imu_buffer.pop_front();
}

void Ekf::sigma_to_imu(const Eigen::Matrix<double, 6, 13> &sigma_points, Eigen::Matrix<double, 4, 13> &Z_pred)
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
  sigma_to_gps(sigma_points, Z_predicted);

  // 4. Compute predicted observation. z_predicted = Zw
  Eigen::Matrix<double, 3, 1> z_predicted = Z_predicted * m_weights_m;

  // 5. Compute uncertainty in transform + observation uncertainty. S = (Z - z) * w * (Z - z)^T + Q
  Eigen::Matrix<double, 3, 13> Z_diff;
  for (int i = 0; i < 13; i++)
  {
    Z_diff.col(i) = Z_predicted.col(i) - z_predicted;
  }
  Eigen::Matrix<double, 3, 3> S = Z_diff * m_weights_c.asDiagonal() * Z_diff.transpose() + m_gps_noise;

  // 6. Compute cross co-relation matrix between state and measurement space. T = (χ - μ) (Z - z)^T * w
  Eigen::Matrix<double, 6, 13> sigma_diff;
  for (int i = 0; i < 6; i++)
  {
    sigma_points.col(i) - m_mu_predicted;
  }
  Eigen::Matrix<double, 6, 3> cross_corelation = sigma_diff * m_weights_c.asDiagonal() * Z_diff.transpose();

  // 7. Compute Kalman Gain. K = T S^-1
  Eigen::Matrix<double, 6, 3> kalman_gain = cross_corelation * S.inverse();

  // 8. Compute corrected mean. μ_corrected = μ_predicted + K(z_observed - z_predicted)
  m_state = m_mu_predicted + kalman_gain * (sensor - z_predicted);

  // 9. Compute corrected covariance. Sigma = Simga_predicted - K S K^T
  m_covariance = m_covar_predicted - kalman_gain * S * kalman_gain.transpose();

  // Publish pose
  publish_pose();

  m_gps_buffer.pop_front();
}

void Ekf::sigma_to_gps(const Eigen::Matrix<double, 6, 13> &sigma_points, Eigen::Matrix<double, 3, 13> &Z_pred)
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
void Ekf::compute_gps_diff(const sensor_msgs::NavSatFixConstPtr &gps)
{
}

inline void Ekf::recalculate_sigma_points(Eigen::Matrix<double, 6, 13> &sigma)
{
  // Calculate offset = gamma * sqrt(covariance) through Cholesky decomposition
  Matrix6d offset{ ((m_state_dims + m_lambda)*m_covar_predicted).llt().matrixL() };
  Matrix6d stacked_mu_predicted;
  stacked_mu_predicted << m_mu_predicted, m_mu_predicted, m_mu_predicted, m_mu_predicted, m_mu_predicted,
      m_mu_predicted;

  // First col is average
  sigma.col(0) = m_mu_predicted;
  // Col 1 -> n is average + gamma * covariance
  sigma.block<6, 6>(0, 1) = stacked_mu_predicted + offset;
  // Col n+1 -> 2n + 1 is average - gamma * covariance
  sigma.block<6, 6>(0, 7) = stacked_mu_predicted - offset;
  //  ROS_INFO_STREAM("Sigma: \n" << std::setprecision(2) << sigma);
}

/**
 * Predicts state at target_time using differential drive motion model
 * @param target_time Time to predict till
 */
void Ekf::prediction_step(const ros::Time &target_time)
{
  // Calculate how much time we need to predict
  ros::Duration dt = target_time - m_last_update_time;

  // Calculate offset = gamma * sqrt(covariance) through Cholesky decomposition
  Matrix6d offset = ((m_state_dims + m_lambda) * m_covariance).llt().matrixL();
  //  ROS_INFO_STREAM("======== Covariance matrix: =========\n" << std::setprecision(3) << m_covariance);

  //  ROS_INFO_STREAM("Offset matrix: \n" << std::setprecision(2) << offset);

  // Calculate Sigma points
  Eigen::Matrix<double, 6, 13> sigma;
  Matrix6d stacked_state;
  stacked_state << m_state, m_state, m_state, m_state, m_state, m_state;
  // First col is average
  sigma.col(0) = m_state;
  // Col 1 -> n is average + gamma * covariance
  sigma.block<6, 6>(0, 1) = stacked_state + offset;
  // Col n+1 -> 2n + 1 is average - gamma * covariance
  sigma.block<6, 6>(0, 7) = stacked_state - offset;

  //  ROS_INFO_STREAM("========== Prediction Step Sigma points ==========\n" << std::setprecision(2) << sigma);

  // Predict until target_time using motion model
  Eigen::Matrix<double, 6, 13> sigma_star;
  motion_model(dt, sigma, sigma_star);
  ROS_INFO_STREAM("=========== Motion Model =========\n" << std::setprecision(6) << sigma_star);
  //  ROS_INFO_STREAM("=========== Weights =========\n" << std::setprecision(6) << m_weights_m.transpose());

  double sum = 0;
  for (int i = 0; i < 13; ++i)
  {
    sum += m_weights_m(i);
  }
  //  ROS_INFO_STREAM("SUM OF WEIGHTS: " << sum);

  sum = 0;
  for (int i = 0; i < 6; ++i)
  {
    sum += ((sigma_star(0, i + 1) * m_weights_m(i + 1)) + (sigma_star(0, i + 7) * m_weights_m(i + 7)));
    ROS_INFO_STREAM("Sum: " << sum);
  }
  sum += (sigma_star(0, 0) * m_weights_m(0));
  ROS_INFO_STREAM("SUM OF X WEIGHTED: " << sum);

  ROS_INFO_STREAM("\n\n");
  // Calculate predicted mean
  m_mu_predicted.setZero();
  for (int i = 0; i < 6; ++i)
  {
    m_mu_predicted =
        m_mu_predicted + ((sigma_star.col(i + 1) * m_weights_m(i + 1)) + (sigma_star.col(i + 7) * m_weights_m(i + 7)));
  }
  m_mu_predicted = m_mu_predicted + (sigma_star.col(0) * m_weights_m(0));
  //  m_mu_predicted = sigma_star * m_weights_m;
  //  ROS_INFO_STREAM("m_weights_m: \n" << std::setprecision(2) << m_weights_m);
  ROS_INFO_STREAM("\n========== m_mu_predicted ==========\n" << std::setprecision(2) << m_mu_predicted);

  // Calculate predicted covariance
  m_covar_predicted.setZero();
  for (int i = 0; i < 13; i++)
  {
    m_covar_predicted +=
        m_weights_c(i) * (sigma_star.col(i) - m_mu_predicted) * (sigma_star.col(i) - m_mu_predicted).transpose();
  }
  // Calculate process noise and not just yeet it
  calculate_process_noise(dt.toSec());

  ROS_INFO_STREAM("========== process noise: ==========\n" << std::setprecision(2) << m_noise_predict);
  m_covar_predicted = m_covar_predicted + m_noise_predict;
  ROS_INFO_STREAM("========== m_covariance_predicted: ==========\n" << std::setprecision(2) << m_covar_predicted);
}

void Ekf::calculate_process_noise(double dt)
{
  m_noise_predict(0, 0) = m_phi1 * pow(dt, 5) / 20;
  m_noise_predict(0, 1) = 0;
  m_noise_predict(0, 2) = m_phi1 * pow(dt, 4) / 8;
  m_noise_predict(0, 3) = m_phi1 * pow(dt, 3) / 6;
  m_noise_predict(0, 4) = 0;  // how to do this lol
  m_noise_predict(0, 5) = 0;  // no pls
  m_noise_predict(1, 0) = 0;
  m_noise_predict(1, 1) = m_phi1 * pow(dt, 5) / 20;
  m_noise_predict(1, 2) = m_phi1 * pow(dt, 4) / 8;
  m_noise_predict(1, 3) = m_phi1 * pow(dt, 3) / 6;
  m_noise_predict(1, 4) = 0;  // how to do this lol
  m_noise_predict(1, 5) = 0;  // no pls
  m_noise_predict(2, 0) = m_phi1 * pow(dt, 4) / 8;
  m_noise_predict(2, 1) = m_phi1 * pow(dt, 4) / 8;
  m_noise_predict(2, 2) = m_phi1 * pow(dt, 3) / 3;
  m_noise_predict(2, 3) = m_phi1 * pow(dt, 2) / 2;
  m_noise_predict(2, 4) = 0;  // how to do this lol
  m_noise_predict(2, 5) = 0;  // no pls
  m_noise_predict(3, 0) = m_phi1 * pow(dt, 3) / 6;
  m_noise_predict(3, 1) = m_phi1 * pow(dt, 3) / 6;
  m_noise_predict(3, 2) = m_phi1 * pow(dt, 2) / 2;
  m_noise_predict(3, 3) = m_phi1 * dt;
  m_noise_predict(3, 4) = 0;  // how to do this lol
  m_noise_predict(3, 5) = 0;
  m_noise_predict(4, 0) = 0;
  m_noise_predict(4, 1) = 0;
  m_noise_predict(4, 2) = 0;
  m_noise_predict(4, 3) = 0;
  //  m_noise_predict(4, 4) = 0;
  m_noise_predict(4, 4) = m_phi2 * pow(dt, 3) / 3;
  m_noise_predict(4, 5) = m_phi2 * pow(dt, 2) / 2;
  m_noise_predict(5, 0) = 0;
  m_noise_predict(5, 1) = 0;
  m_noise_predict(5, 2) = 0;
  m_noise_predict(5, 3) = 0;
  m_noise_predict(5, 4) = m_phi2 * pow(dt, 2) / 2;
  m_noise_predict(5, 5) = m_phi2 * dt;
}

void Ekf::motion_model(const ros::Duration &dt, const Eigen::Matrix<double, 6, 13> &sigma,
                       Eigen::Matrix<double, 6, 13> &sigma_star)
{
  // Calculate in rows?
  // x, y, v, a, theta, theta'
  //
  // x = x + (v + a/4 * dt) * cos(theta + dt * theta' / 2) * dt
  // y = y + (v + a/4 * dt) * sin (theta + dt * theta' / 2) * dt
  // v = v + dt * a (But we use v = v + dt * a/2 for x, y calculation)
  // a = a
  // theta = theta + dt * theta' (But we use theta = theta + dt * theta / 2)
  // theta' = theta'

  ROS_INFO_STREAM("Duration: " << dt.toSec());
  ROS_INFO_STREAM("sigma input in motion_model: \n" << std::setprecision(2) << sigma);
  Eigen::Matrix<double, 1, 13> mid_theta = (theta(sigma) + dt.toSec() / 2 * theta_dot(sigma));
  theta(sigma_star) = theta(sigma) + dt.toSec() * theta(sigma);
  theta_dot(sigma_star) = theta_dot(sigma);

  a(sigma_star) = a(sigma);
  v(sigma_star) = v(sigma) + dt.toSec() * a(sigma);
  Eigen::Matrix<double, 1, 13> dpos = (v(sigma) + dt.toSec() * a(sigma) / 4);

  x(sigma_star) = x(sigma) + (dpos.array() * mid_theta.array().cos() * dt.toSec()).matrix();
  y(sigma_star) = y(sigma) + (dpos.array() * mid_theta.array().sin() * dt.toSec()).matrix();
}

Ekf::Sensor Ekf::next_sensor()
{
  if (m_gps_buffer.empty() || m_imu_buffer.empty() || m_odom_buffer.empty())
  {
    if (m_gps_buffer.empty() && m_imu_buffer.empty() && !m_odom_buffer.empty())
    {
      return Sensor::odom;
    }
    return Sensor::none;
  }
  if (STAMP(m_odom_buffer.front()) < STAMP(m_imu_buffer.front()) &&
      STAMP(m_odom_buffer.front()) < STAMP(m_gps_buffer.front()))
  {
    return Sensor::odom;
  }
  else if (STAMP(m_imu_buffer.front()) < STAMP(m_odom_buffer.front()) &&
           STAMP(m_imu_buffer.front()) < STAMP(m_gps_buffer.front()))
  {
    return Sensor::imu;
  }
  else
  {
    return Sensor::gps;
  }
}

void Ekf::publish_pose() const
{
  // Publish TF Transform
  static tf::TransformBroadcaster br;
  tf::Matrix3x3 rot;
  // x y v a theta theta'
  rot.setRPY(0, 0, m_state(4));
  tf::Vector3 coords{ m_state(0), m_state(1), 0 };
  tf::Transform transform;
  transform.setBasis(rot);
  transform.setOrigin(coords);
  br.sendTransform(tf::StampedTransform{ transform, ros::Time::now(), m_odom_frame, m_base_frame });
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "UKF");
  Ekf ukf;
}