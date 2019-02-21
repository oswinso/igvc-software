#ifndef PROJECT_MADGWICK_FILTER_H
#define PROJECT_MADGWICK_FILTER_H

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

class Madgwick_filter
{
public:
  Madgwick_filter();

private:
  double m_gain;
  double m_zeta;

  double q0, q1, q2, q3;
  double w_bx_, w_by_, w_bz_;

public:
  void madgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my,
                          double mz, double dt);
  void madgwickAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az, double dt);
  bool compute_orientation(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& E,
                           geometry_msgs::Quaternion& orientation);
  void set_orientation(double x, double y, double z, double w);
  void get_orientation(double& x, double& y, double& z, double& w);

  void setAlgorithmicGain(double gain);
  void setDriftBiasGain(double zeta);
};

#endif  // PROJECT_MADGWICK_FILTER_H
