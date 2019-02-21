#ifndef PROJECT_MADGWICK_FILTER_NODE_H
#define PROJECT_MADGWICK_FILTER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "madgwick_filter.h"

class Madgwick_filter_node
{
  using Imu_msg = sensor_msgs::Imu;
  using Mag_msg = sensor_msgs::MagneticField;

  using Sync_policy = message_filters::sync_policies::ApproximateTime<Imu_msg, Mag_msg>;
  using Synchronizer = message_filters::Synchronizer<Sync_policy>;
  using Imu_subscriber = message_filters::Subscriber<Imu_msg>;
  using Mag_subscriber = message_filters::Subscriber<Mag_msg>;

public:
  Madgwick_filter_node(ros::NodeHandle& ph, const ros::NodeHandle& pNh);

private:
  double m_roll_variance;
  double m_pitch_variance;
  double m_yaw_variance;

  int m_queue_size;
  bool m_debug;

  std::unique_ptr<Imu_subscriber> m_imu_subscriber;
  std::unique_ptr<Mag_subscriber> m_mag_subscriber;
  std::unique_ptr<Synchronizer> m_synchronizer;

  ros::Publisher m_filtered_pub;
  ros::Publisher m_rpy_pub;

//  std::mutex m_mutex;
  bool m_initialized;
  ros::Time m_last_time;

  Madgwick_filter m_filter;

  void callback(const Imu_msg::ConstPtr& imu_msg_raw, const Mag_msg::ConstPtr& mag_msg);
  void publish_filtered_msg(const Imu_msg::ConstPtr& filtered_msg);

public:
};

#endif  // PROJECT_MADGWICK_FILTER_NODE_H
