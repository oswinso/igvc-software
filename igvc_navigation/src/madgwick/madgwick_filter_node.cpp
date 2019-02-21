#include "madgwick_filter_node.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <igvc_utils/NodeUtils.hpp>

Madgwick_filter_node::Madgwick_filter_node(ros::NodeHandle& nh, const ros::NodeHandle& pNh) : m_initialized{false}
{
  std::string imu_topic, mag_topic, output_topic;
  igvc::getParam(pNh, "topics/imu", imu_topic);
  igvc::getParam(pNh, "topics/mag", mag_topic);
  igvc::getParam(pNh, "topics/output", output_topic);
  igvc::getParam(pNh, "variance/roll", m_roll_variance);
  igvc::getParam(pNh, "variance/pitch", m_pitch_variance);
  igvc::getParam(pNh, "variance/yaw", m_yaw_variance);
  igvc::getParam(pNh, "node/debug", m_debug);
  igvc::getParam(pNh, "node/queue_size", m_queue_size);

  double gain, zeta;
  igvc::getParam(pNh, "parameters/gain", gain);
  igvc::getParam(pNh, "parameters/zeta", zeta);

  m_filtered_pub = nh.advertise<sensor_msgs::Imu>(output_topic, 5);
  m_mag_subscriber.reset(new Mag_subscriber(nh, mag_topic, m_queue_size));
  m_imu_subscriber.reset(new Imu_subscriber(nh, imu_topic, m_queue_size));

  m_synchronizer.reset(new Synchronizer(Sync_policy(m_queue_size), *m_imu_subscriber, *m_mag_subscriber));
  m_synchronizer->registerCallback(boost::bind(&Madgwick_filter_node::callback, this, _1, _2));

  m_filter = Madgwick_filter{};
  m_filter.setAlgorithmicGain(gain);
  m_filter.setDriftBiasGain(zeta);
  ROS_INFO("Imu filter gain: %f", gain);
  ROS_INFO("Gyro drift bias: %f", zeta);
}

void Madgwick_filter_node::callback(const sensor_msgs::Imu_<std::allocator<void>>::ConstPtr& imu_msg_raw,
                                    const sensor_msgs::MagneticField_<std::allocator<void>>::ConstPtr& mag_msg)
{
  const geometry_msgs::Vector3& angular_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_accel = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3& mag_field = mag_msg->magnetic_field;

  ros::Time cur_time = imu_msg_raw->header.stamp;

  if (!m_initialized)
  {
    geometry_msgs::Quaternion init_quat;
    if (!m_filter.compute_orientation(lin_accel, mag_field, init_quat))
    {
      ROS_WARN_THROTTLE(5.0, "The IMU seems to be in free fall, cannot determine gravity direction.");
      return;
    }
    m_filter.set_orientation(init_quat.x, init_quat.y, init_quat.z, init_quat.w);

    ROS_INFO("First pair of IMU and mag messages received.");
    m_last_time = cur_time;
    m_initialized = true;
  }

  // Determine dt
  double dt = (cur_time - m_last_time).toSec();
  m_last_time = cur_time;

  m_filter.madgwickAHRSupdate(angular_vel.x, angular_vel.y, angular_vel.z, lin_accel.x, lin_accel.y, lin_accel.z,
                              mag_field.x, mag_field.y, mag_field.z, dt);

  publish_filtered_msg(imu_msg_raw);
}

void Madgwick_filter_node::publish_filtered_msg(const Imu_msg::ConstPtr& raw_msg)
{
  double q0, q1, q2, q3;
  m_filter.get_orientation(q0, q1, q2, q3);

  // create and publish filtered IMU message
  boost::shared_ptr<Imu_msg> imu_msg = boost::make_shared<Imu_msg>(*raw_msg);

  imu_msg->orientation.w = q0;
  imu_msg->orientation.x = q1;
  imu_msg->orientation.y = q2;
  imu_msg->orientation.z = q3;

  imu_msg->orientation_covariance[0] = m_roll_variance;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = m_pitch_variance;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = m_yaw_variance;

  m_filtered_pub.publish(imu_msg);

  if (m_debug)
  {
    geometry_msgs::Vector3Stamped rpy;
    tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0)).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

    rpy.header = raw_msg->header;
    m_rpy_pub.publish(rpy);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MadgwickFilter");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");
  Madgwick_filter_node node{nh, pNh};
  ros::spin();
  return 0;
}
