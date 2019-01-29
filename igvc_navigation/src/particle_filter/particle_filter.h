#pragma once
#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include <geometry_msgs/TwistWithCovariance.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <igvc_utils/RobotState.hpp>
#include "igvc_navigation/hsluv.h"

#include <random>
#include "octomapper.h"
#include "scanmatcher.h"

struct Particle
{
  RobotState state;
  pc_map_pair pair;
  float weight{ 0 };
  Particle(const Particle& p) : state{ p.state }, pair{}, weight(p.weight)
  {
    pair.octree = std::unique_ptr<octomap::OcTree>(new octomap::OcTree(*p.pair.octree));
    pair.map = nullptr;
  }
  Particle() : state{}, pair{}, weight{ 1 }
  {
  }
};

class Particle_filter
{
public:
  Particle_filter(const ros::NodeHandle& pNh);
  void initialize_particles(const tf::Transform& pose);
  void update(const tf::Transform& delta_pose, const ros::Duration& delta_t,
                               const pcl::PointCloud<pcl::PointXYZ>& pc,
                               const tf::Transform& lidar_to_base);
  int length_x()
  {
    return m_octomapper.length_x();
  }
  int width_y()
  {
    return m_octomapper.width_y();
  }
  int start_x()
  {
    return m_octomapper.start_x();
  }
  int start_y()
  {
    return m_octomapper.start_y();
  }
  double resolution()
  {
    return m_octomapper.resolution();
  }
  int m_best_idx;
  std::vector<Particle> m_particles;

private:
  void resample_particles();
  void map_weight_to_rgb(float weight, double* r, double* g, double* b);
  void visualize_key(const octomap::KeySet freeSet, const octomap::KeySet occSet, const pc_map_pair& pair,
                     uint64_t stamp);
  inline double gauss(double variance);

  bool m_debug;
  bool m_use_scanmatch;
  bool m_is_3d;
  int m_num_particles{};
  int m_scanmatch_point_thresh;
  float m_inverse_resample_threshold;
  float m_total_weight;
  double m_thresh_still;
  double m_viz_hue_start, m_viz_hue_end;
  double m_viz_sat_start, m_viz_sat_end;
  double m_viz_light_start, m_viz_light_end;
  double m_variance_x, m_variance_y, m_variance_yaw;
  double m_scanmatch_variance_x, m_scanmatch_variance_y, m_scanmatch_variance_yaw;

  static constexpr double m_viz_hue_max = 360;
  static constexpr double m_viz_sat_max = 100;
  static constexpr double m_viz_light_max = 100;

  ros::NodeHandle pNh;
  ros::Publisher m_particle_pub;
  ros::Publisher m_ground_pub;
  ros::Publisher m_nonground_pub;
  ros::Publisher m_num_eff_particles_pub;
  Octomapper m_octomapper;
  Scanmatcher m_scanmatcher;
};
#endif  // PROJECT_PARTICLE_FILTER_H
