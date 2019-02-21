#include "madgwick_filter.h"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static double invSqrt(double x)
{
  double xhalf = 0.5 * x;
  union {
    double x;
    int i;
  } u;
  u.x = x;
  u.i = 0x5f3759d - (u.i >> 1);
  /* doublehe next line can be repeated any number of times to increase accuracy */
  u.x = u.x * (1.5 - xhalf * u.x * u.x);
  return u.x;
}

static inline void normalizeVector(double& vx, double& vy, double& vz)
{
  double recipNorm = invSqrt(vx * vx + vy * vy + vz * vz);
  vx *= recipNorm;
  vy *= recipNorm;
  vz *= recipNorm;
}

static inline double normVectorWithNorm(double vx, double vy, double vz)
{
  double norm = sqrt(vx*vx + vy*vy + vz*vz);
  double inv = 1.0 / norm;
  vx *= inv;
  vy *= inv;
  vz *= inv;
  return norm;
}

static inline void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
  double recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

static inline void rotateAndScaleVector(double q0, double q1, double q2, double q3, double _2dx, double _2dy,
                                        double _2dz, double& rx, double& ry, double& rz)
{
  // result is half as long as input
  rx = _2dx * (0.5f - q2 * q2 - q3 * q3) + _2dy * (q0 * q3 + q1 * q2) + _2dz * (q1 * q3 - q0 * q2);
  ry = _2dx * (q1 * q2 - q0 * q3) + _2dy * (0.5f - q1 * q1 - q3 * q3) + _2dz * (q0 * q1 + q2 * q3);
  rz = _2dx * (q0 * q2 + q1 * q3) + _2dy * (q2 * q3 - q0 * q1) + _2dz * (0.5f - q1 * q1 - q2 * q2);
}

static inline void compensateGyroDrift(double q0, double q1, double q2, double q3, double s0, double s1, double s2,
                                       double s3, double dt, double zeta, double& w_bx, double& w_by, double& w_bz,
                                       double& gx, double& gy, double& gz)
{
  // w_err = 2 q x s
  double w_err_x = 2.0f * q0 * s1 - 2.0f * q1 * s0 - 2.0f * q2 * s3 + 2.0f * q3 * s2;
  double w_err_y = 2.0f * q0 * s2 + 2.0f * q1 * s3 - 2.0f * q2 * s0 - 2.0f * q3 * s1;
  double w_err_z = 2.0f * q0 * s3 - 2.0f * q1 * s2 + 2.0f * q2 * s1 - 2.0f * q3 * s0;

  w_bx += w_err_x * dt * zeta;
  w_by += w_err_y * dt * zeta;
  w_bz += w_err_z * dt * zeta;

  gx -= w_bx;
  gy -= w_by;
  gz -= w_bz;
}

static inline void orientationChangeFromGyro(double q0, double q1, double q2, double q3, double gx, double gy,
                                             double gz, double& qDot1, double& qDot2, double& qDot3, double& qDot4)
{
  // Rate of change of quaternion from gyroscope
  // See EQ 12
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
}

static inline void addGradientDescentStep(double q0, double q1, double q2, double q3, double _2dx, double _2dy,
                                          double _2dz, double mx, double my, double mz, double& s0, double& s1,
                                          double& s2, double& s3)
{
  double f0, f1, f2;

  // Gradient decent algorithm corrective step
  // EQ 15, 21
  rotateAndScaleVector(q0, q1, q2, q3, _2dx, _2dy, _2dz, f0, f1, f2);

  f0 -= mx;
  f1 -= my;
  f2 -= mz;

  // EQ 22, 34
  // Jt * f
  s0 += (_2dy * q3 - _2dz * q2) * f0 + (-_2dx * q3 + _2dz * q1) * f1 + (_2dx * q2 - _2dy * q1) * f2;
  s1 += (_2dy * q2 + _2dz * q3) * f0 + (_2dx * q2 - 2.0f * _2dy * q1 + _2dz * q0) * f1 +
        (_2dx * q3 - _2dy * q0 - 2.0f * _2dz * q1) * f2;
  s2 += (-2.0f * _2dx * q2 + _2dy * q1 - _2dz * q0) * f0 + (_2dx * q1 + _2dz * q3) * f1 +
        (_2dx * q0 + _2dy * q3 - 2.0f * _2dz * q2) * f2;
  s3 += (-2.0f * _2dx * q3 + _2dy * q0 + _2dz * q1) * f0 + (-_2dx * q0 - 2.0f * _2dy * q3 + _2dz * q2) * f1 +
        (_2dx * q1 + _2dy * q2) * f2;
}

static inline void compensateMagneticDistortion(double q0, double q1, double q2, double q3, double mx, double my,
                                                double mz, double& _2bxy, double& _2bz)
{
  double hx, hy, hz;
  // Reference direction of Earth's magnetic field (See EQ 46)
  rotateAndScaleVector(q0, -q1, -q2, -q3, mx, my, mz, hx, hy, hz);

  _2bxy = 4.0f * sqrt(hx * hx + hy * hy);
  _2bz = 4.0f * hz;
}

Madgwick_filter::Madgwick_filter()
  : q0(1.0), q1(0.0), q2(0.0), q3(0.0), w_bx_(0.0), w_by_(0.0), w_bz_(0.0), m_zeta(0.0), m_gain(0.0)
{
}

void Madgwick_filter::set_orientation(double x, double y, double z, double w)
{
  q0 = x;
  q1 = y;
  q2 = z;
  q3 = w;
}

void Madgwick_filter::madgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx,
                                         double my, double mz, double dt)
{
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double _2bz, _2bxy;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if (!std::isfinite(mx) || !std::isfinite(my) || !std::isfinite(mz))
  {
    madgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    normalizeVector(ax, ay, az);

    // Normalise magnetometer measurement
    normalizeVector(mx, my, mz);

    // Compensate for magnetic distortion
    compensateMagneticDistortion(q0, q1, q2, q3, mx, my, mz, _2bxy, _2bz);

    // Gradient decent algorithm corrective step
    s0 = 0.0;
    s1 = 0.0;
    s2 = 0.0;
    s3 = 0.0;
    // Gravity: [0, 0, 1]
    addGradientDescentStep(q0, q1, q2, q3, 0.0, 0.0, 2.0, ax, ay, az, s0, s1, s2, s3);

    // Earth magnetic field: = [0, bxy, bz]
    addGradientDescentStep(q0, q1, q2, q3, 0.0, _2bxy, _2bz, mx, my, mz, s0, s1, s2, s3);
    normalizeQuaternion(s0, s1, s2, s3);

    // compute gyro drift bias
    compensateGyroDrift(q0, q1, q2, q3, s0, s1, s2, s3, dt, m_zeta, w_bx_, w_by_, w_bz_, gx, gy, gz);

    orientationChangeFromGyro(q0, q1, q2, q3, gx, gy, gz, qDot1, qDot2, qDot3, qDot4);

    // Apply feedback step
    qDot1 -= m_gain * s0;
    qDot2 -= m_gain * s1;
    qDot3 -= m_gain * s2;
    qDot4 -= m_gain * s3;
  }
  else
  {
    orientationChangeFromGyro(q0, q1, q2, q3, gx, gy, gz, qDot1, qDot2, qDot3, qDot4);
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  normalizeQuaternion(q0, q1, q2, q3);
}

void Madgwick_filter::madgwickAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az, double dt)
{
  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;

  // Rate of change of quaternion from gyroscope
  orientationChangeFromGyro(q0, q1, q2, q3, gx, gy, gz, qDot1, qDot2, qDot3, qDot4);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
  {
    // Normalise accelerometer measurement
    normalizeVector(ax, ay, az);

    // Gradient decent algorithm corrective step
    s0 = 0.0;
    s1 = 0.0;
    s2 = 0.0;
    s3 = 0.0;
    addGradientDescentStep(q0, q1, q2, q3, 0.0, 0.0, 2.0, ax, ay, az, s0, s1, s2, s3);

    normalizeQuaternion(s0, s1, s2, s3);

    // Apply feedback step
    qDot1 -= m_gain * s0;
    qDot2 -= m_gain * s1;
    qDot3 -= m_gain * s2;
    qDot4 -= m_gain * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  normalizeQuaternion(q0, q1, q2, q3);
}

static inline void crossProduct(double ax, double ay, double az, double bx, double by, double bz, double& rx, double& ry, double& rz)
{
  rx = ay * bz - az * by;
  ry = az * bx - ax * bz;
  rz = ax * by - ay * bx;
}

bool Madgwick_filter::compute_orientation(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& E,
                                         geometry_msgs::Quaternion& orientation)
{
  double Hx, Hy, Hz;
  double Mx, My, Mz;
  double normH, invH, invA;

  // A: pointing up
  double Ax = A.x, Ay = A.y, Az = A.z;

  // E: pointing down/north
  double Ex = E.x, Ey = E.y, Ez = E.z;

  // H: vector horizontal, pointing east
  // H = E x A
  crossProduct(Ex, Ey, Ez, Ax, Ay, Az, Hx, Hy, Hz);

  // normalize H
  normH = normVectorWithNorm(Hx, Hy, Hz);
  if (normH < 1E-7)
  {
    // device is close to free fall (or in space?), or close to
    // magnetic north pole.
    // mag in double => doublehreshold 1E-7, typical values are  > 1E-5.
    return false;
  }

  // normalize A
  normalizeVector(Ax, Ay, Az);

  // M: vector horizontal, pointing north
  // M = A x H
  crossProduct(Ax, Ay, Az, Hx, Hy, Hz, Mx, My, Mz);

  // Create matrix for basis transformation
  tf2::Matrix3x3 R;
  // vector space world W:
  // Basis: bwx (1,0,0) east, bwy (0,1,0) north, bwz (0,0,1) up
  // vector space local L:
  // Basis: H, M , A
  // W(1,0,0) => L(H)
  // W(0,1,0) => L(M)
  // W(0,0,1) => L(A)

  // R: doubleransform Matrix local => world equals basis of L, because basis of W is I
  R[0][0] = Hx;
  R[0][1] = Mx;
  R[0][2] = Ax;
  R[1][0] = Hy;
  R[1][1] = My;
  R[1][2] = Ay;
  R[2][0] = Hz;
  R[2][1] = Mz;
  R[2][2] = Az;
  // Matrix.getRotation assumes vector rotation, but we're using
  // coordinate systems. doublehus negate rotation angle (inverse).
  tf2::Quaternion q;
  R.getRotation(q);
  tf2::convert(q.inverse(), orientation);
  return true;
}

void Madgwick_filter::get_orientation(double& x, double& y, double& z, double& w) {
  x = q0;
  y = q1;
  z = q2;
  w = q3;
}
void Madgwick_filter::setDriftBiasGain(double zeta) {
  m_zeta = zeta;
}
void Madgwick_filter::setAlgorithmicGain(double gain) {
  m_gain = gain;
}
