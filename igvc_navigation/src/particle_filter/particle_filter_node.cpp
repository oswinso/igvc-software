#include <cv_bridge/cv_bridge.h>

#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/RobotState.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/circular_buffer.hpp>

#include "particle_filter.h"

#include <ros/ros.h>
#include <signal.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <gperftools/profiler.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <random>

class ParticleFilterNode {
public:

  ParticleFilterNode(const ros::NodeHandle &pNh) : m_particle_filter{
      std::unique_ptr<Particle_filter>(new Particle_filter(pNh))}, m_lidar_transform{nullptr},
                                                   m_last_time{ros::Time::now()}, m_tf_listener{} {
    int pc_buf_size, pose_buf_size, icp_match_history_length;
    igvc::getParam(pNh, "update_time_threshold", m_update_time_thresh);
    igvc::getParam(pNh, "start_x", m_start_x);
    igvc::getParam(pNh, "start_y", m_start_y);
    igvc::getParam(pNh, "start_z", m_start_z);
    igvc::getParam(pNh, "particle_filter_buffer_size", pc_buf_size);
    igvc::getParam(pNh, "pose_buffer_size", pose_buf_size);
    igvc::param(pNh, "debug", m_debug, false);
    igvc::param(pNh, "profile", m_profile, false);
    igvc::getParam(pNh, "lidar_topic", m_lidar_topic);
    igvc::getParam(pNh, "fused_topic", m_fused_topic);
    igvc::getParam(pNh, "odometry_frame", m_odom_frame);
    igvc::getParam(pNh, "base_frame", m_base_frame);
    igvc::getParam(pNh, "lidar_frame", m_lidar_frame);
    igvc::getParam(pNh, "parent_frame", m_parent_frame);
    igvc::getParam(pNh, "child_frame", m_child_frame);
    igvc::getParam(pNh, "voxel_grid_size", m_voxel_grid_size);
    igvc::getParam(pNh, "filter/min_distance", m_filter_min);
    igvc::getParam(pNh, "filter/max_distance", m_filter_max);
    igvc::getParam(pNh, "icp/match_history_length", icp_match_history_length);
    igvc::getParam(pNh, "particle_filter/variance/x", m_variance_x);
    igvc::getParam(pNh, "particle_filter/variance/y", m_variance_y);
    igvc::getParam(pNh, "particle_filter/variance/yaw", m_variance_yaw);
  }

  void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc);

  void pose_callback(const nav_msgs::OdometryConstPtr &pose);

  bool m_profile = false;
  tf::TransformListener m_tf_listener;
  std::unique_ptr<Particle_filter> m_particle_filter;
  ros::Publisher m_map_pub;
  ros::Publisher m_map_pcl_debug_pub;
  std::string m_base_frame, m_lidar_frame;
  std::string m_lidar_topic, m_fused_topic, m_odom_frame;
  std::string m_parent_frame, m_child_frame;

private:
  void check_update();

  void update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& scan);
  inline double gauss(double variance);

  void publish(const ros::Time &stamp);

  void get_lidar_transform();

  double m_variance_x, m_variance_y, m_variance_yaw;
  bool m_debug{};
  bool m_initialised = false;
  double m_voxel_grid_size{};
  double m_update_time_thresh{}, m_start_x{}, m_start_y{}, m_start_z{};
  std::shared_ptr<tf::Stamped<tf::Pose>> m_last_pose;
  double m_filter_min{}, m_filter_max{};
  ros::Time m_last_time;
  cv_bridge::CvImage m_img_bridge;
  RobotState m_delta_pose;
  boost::circular_buffer<nav_msgs::OdometryConstPtr> m_pose_buf;
  boost::shared_ptr<tf::StampedTransform> m_lidar_transform;
};

std::unique_ptr<ParticleFilterNode> pf_node;

void node_cleanup(int sig) {
  pf_node->m_particle_filter.reset();
  pf_node.reset();
  ros::shutdown();
}

void ParticleFilterNode::pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {
  update(pc);
}

void ParticleFilterNode::pose_callback(const nav_msgs::OdometryConstPtr &pose) {

  if (!m_initialised) {
    m_last_time = pose->header.stamp;
    m_delta_pose.set_x(pose->pose.pose.position.x);
    m_delta_pose.set_y(pose->pose.pose.position.y);
    tf::Quaternion rot{pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w};
    m_delta_pose.transform.setRotation(rot);
    m_initialised = true;
  }
  ROS_INFO("1");
  ros::Duration delta_t = pose->header.stamp - m_last_time;
  double noisy_x = pose->twist.twist.linear.x;
  double noisy_y = pose->twist.twist.linear.y;
  double noisy_yaw = pose->twist.twist.angular.z;
  double cur_yaw = m_delta_pose.yaw();
  double new_x = noisy_x * cos(cur_yaw) - noisy_y * sin(cur_yaw);
  double new_y = noisy_x * sin(cur_yaw) + noisy_y * cos(cur_yaw);
  ROS_INFO("2");
  m_delta_pose.set_x(m_delta_pose.x() + delta_t.toSec() * new_x);
  m_delta_pose.set_y(m_delta_pose.y() + delta_t.toSec() * new_y);
  m_delta_pose.set_yaw(m_delta_pose.yaw() + delta_t.toSec() * noisy_yaw);
  m_delta_pose.transform.getRotation().normalize();
  m_last_time = pose->header.stamp;
  ROS_INFO("3");
}

/**
 * Performs one iteration of particle filter using the indices passed
 * @param pose_idx
 * @param pc_idx
 */
void ParticleFilterNode::update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& scan) {
  ROS_INFO("4");
  static tf::TransformBroadcaster br;
  ROS_INFO("5");
  br.sendTransform(
      tf::StampedTransform(m_delta_pose.transform, ros::Time::now(), m_parent_frame, m_child_frame));
  ROS_INFO("6");
  return;
}
//  ros::Time stamp;
//  pcl_conversions::fromPCL(scan->header.stamp, stamp);
//
//  // On first run, initialize particles with the pose
//  if (!m_initialised) {
//    m_particle_filter->initialize_particles(m_delta_pose.transform);
//    m_last_pose = std::make_shared<tf::Stamped<tf::Pose>>(m_delta_pose);
//    m_last_time = stamp;
//    m_initialised = true;
//    ROS_INFO("Initializing particle filter");
//  }
//
//  // TODO: Maybe move this to a lidar filtering node?
//  // Get static transform from lidar frame to base frame
//  get_lidar_transform();
//
//  // TODO: Make new node to filter pc instead of doing it here
//  pcl::PointCloud<pcl::PointXYZ> filtered_pcl;
//  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
////  ROS_INFO_STREAM("Filtered size (0): " << m_pc_buf[pc_idx]->size() << " with leaf size " << m_voxel_grid_size);
//  voxel_grid.setInputCloud(scan);
//  voxel_grid.setLeafSize(m_voxel_grid_size, m_voxel_grid_size, m_voxel_grid_size);
//  voxel_grid.filter(filtered_pcl);
//
//  float distance;
//  pcl::PointCloud<pcl::PointXYZ> filtered_2;
////  ROS_INFO_STREAM("Filtered size (1): " << filtered_pcl.size());
//  for (const auto& point : filtered_pcl)
//  {
//    distance = point.x * point.x + point.y * point.y + point.z * point.z;
//    if (distance > m_filter_min && distance < m_filter_max*m_filter_max) {
//      filtered_2.push_back(point);
//    }
//  }
////  ROS_INFO_STREAM("Filtered size (2): " << filtered_2.size());
//
//  // Transform cloud to base frame
//  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
////  pcl_ros::transformPointCloud(filtered, transformed_pc, *m_lidar_transform);
//  pcl_ros::transformPointCloud(filtered_2, *transformed_pc, *m_lidar_transform);
//
//  m_particle_filter->update(m_delta_pose, *transformed_pc, *m_lidar_transform);
//
//  // Publish newest iteration of particle filter
//  publish(stamp);
////}

void ParticleFilterNode::publish(const ros::Time &stamp) {
  // Publish map form best particle
  igvc_msgs::map message;    // >> message to be sent
  sensor_msgs::Image image;  // >> image in the message
  m_img_bridge = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8,
                                    *m_particle_filter->m_particles[m_particle_filter->m_best_idx].pair.map);
  m_img_bridge.toImageMsg(image);  // from cv_bridge to sensor_msgs::Image

  // Setup message
  double resolution = m_particle_filter->resolution();
  message.header.frame_id = "/odom";
  message.image = image;
  message.length = m_particle_filter->length_x();
  message.width = m_particle_filter->width_y();
  message.resolution = resolution;
  message.orientation = m_particle_filter->m_particles[m_particle_filter->m_best_idx].state.yaw();
  message.x = std::round(m_particle_filter->m_particles[m_particle_filter->m_best_idx].state.x() / resolution) + m_particle_filter->start_x();
  message.y = std::round(m_particle_filter->m_particles[m_particle_filter->m_best_idx].state.y() / resolution) + m_particle_filter->start_y();
  message.x_initial = m_particle_filter->start_x();
  message.y_initial = m_particle_filter->start_y();

  // Publish map
  m_map_pub.publish(message);

  // Publish tf transform
  static tf::TransformBroadcaster br;
  br.sendTransform(
      tf::StampedTransform(m_particle_filter->m_particles[m_particle_filter->m_best_idx].state.transform, stamp, m_parent_frame, m_child_frame));

  if (m_debug) {
    uint64 pc_stamp;
    pcl_conversions::toPCL(stamp, pc_stamp);
    pcl::PointCloud<pcl::PointXYZRGB> debug_pcl =
        pcl::PointCloud<pcl::PointXYZRGB>();
    for (int i = 0; i < m_particle_filter->length_x(); i++) {
      for (int j = 0; j < m_particle_filter->width_y(); j++) {
        pcl::PointXYZRGB p;
        uchar prob = m_particle_filter->m_particles[m_particle_filter->m_best_idx].pair.map->at<uchar>(i, j);
        if (prob > 127) {
          p = pcl::PointXYZRGB();
          p.x = (i - m_particle_filter->length_x() / 2) * resolution; // TODO: Changes these to use start_x
          p.y = (j - m_particle_filter->width_y() / 2) * resolution;
          p.r = 0;
          p.g = static_cast<uint8_t>((prob - 127) * 2);
          p.b = 0;
          debug_pcl.points.push_back(p);
        } else if (prob < 127) {
          p = pcl::PointXYZRGB();
          p.x = (i - m_particle_filter->length_x() / 2) * resolution;
          p.y = (j - m_particle_filter->width_y() / 2) * resolution;
          p.r = 0;
          p.g = 0;
          p.b = static_cast<uint8_t>((127 - prob) * 2);
          debug_pcl.points.push_back(p);
        }
        // Set x y coordinates as the center of the grid cell.
      }
    }
    debug_pcl.header.frame_id = "/odom";
    debug_pcl.header.stamp = pc_stamp;
//    ROS_INFO_STREAM("Size: " << fromOcuGrid->points.size() << " / " << (width_x * length_y));
    m_map_pcl_debug_pub.publish(debug_pcl);
  }
}

void ParticleFilterNode::get_lidar_transform() {
  if (m_lidar_transform == nullptr) {
    if (m_tf_listener.waitForTransform(m_base_frame, m_lidar_frame, ros::Time(0), ros::Duration(2))) {
      ROS_INFO_STREAM("Getting static transform from " << m_lidar_frame << " to " << m_base_frame);
      m_lidar_transform = boost::make_shared<tf::StampedTransform>();
      tf::StampedTransform t;
      m_tf_listener.lookupTransform(m_base_frame, m_lidar_frame, ros::Time(0), t);
      *m_lidar_transform = t;
    } else {
      ROS_ERROR_STREAM("Error getting static transform from " << m_lidar_frame << " to " << m_base_frame);
    }
  }
}

double ParticleFilterNode::gauss(double variance)
{
  static std::mt19937 gen{ std::random_device{}() };
  std::normal_distribution<> dist(0, variance);

  return dist(gen);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  signal(SIGINT, node_cleanup);

  pf_node = std::unique_ptr<ParticleFilterNode>(
      new ParticleFilterNode(pNh));

  ros::Subscriber pc_sub = nh.subscribe(pf_node->m_lidar_topic, 1, &ParticleFilterNode::pc_callback, pf_node.get());
  ros::Subscriber pose_sub = nh.subscribe(pf_node->m_fused_topic, 1, &ParticleFilterNode::pose_callback, pf_node.get());

  pf_node->m_map_pub = nh.advertise<igvc_msgs::map>("/map", 1);
  pf_node->m_map_pcl_debug_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/particle_filter/map_pcl_debug", 1);
  ros::spin();

  return 0;
}

