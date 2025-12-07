#include "Quaternion.h"
#include <cmath>
#include <stdexcept>  // for std::runtime_error

// -------- Quaternion methods --------

Quaternion::Quaternion()
    : qw(1.0), qx(0.0), qy(0.0), qz(0.0) {}

Quaternion::Quaternion(double w_, double x_, double y_, double z_)
    : qw(w_), qx(x_), qy(y_), qz(z_) {}

void Quaternion::Q_to_Euler(double &roll, double &pitch, double &yaw) const {
    double sinr = 2 * (qw * qx + qy * qz);
    double cosr = 1 - 2 * (qx * qx + qy * qy);
    roll = std::atan2(sinr, cosr);

    double sinp = std::sqrt(1 + 2 * (qw * qy - qx * qz));
    double cosp = std::sqrt(1 - 2 * (qw * qy - qx * qz));
    pitch = -M_PI / 2 + 2 * std::atan2(sinp, cosp);

    double siny = 2 * (qw * qz + qx * qy);
    double cosy = 1 - 2 * (qy * qy + qz * qz);
    yaw = std::atan2(siny, cosy);
}

Eigen::Matrix3d Quaternion::Q_to_Rotation() const {
    Eigen::Matrix3d R;

    double w = qw, x = qx, y = qy, z = qz;

    R << 1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),
         2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),
         2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y);

    return R;
}

double Quaternion::norm() const {
    return std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
}

void Quaternion::normalize() {
    double n = norm();
    if (n == 0.0) {
        throw std::runtime_error("Cannot normalize zero quaternion");
    }
    qw /= n;
    qx /= n;
    qy /= n;
    qz /= n;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(qw, -qx, -qy, -qz);
}

Quaternion Quaternion::inverse() const {
    double n2 = norm() * norm();
    if (n2 == 0.0) {
        throw std::runtime_error("Cannot invert zero quaternion");
    }
    Quaternion q_conj = conjugate();
    return Quaternion(
        q_conj.qw / n2,
        q_conj.qx / n2,
        q_conj.qy / n2,
        q_conj.qz / n2
    );
}

Quaternion Quaternion::quat_mul(const Quaternion& q1, const Quaternion& q2) {
    double qw_new = q1.qw * q2.qw
                  - q1.qx * q2.qx
                  - q1.qy * q2.qy
                  - q1.qz * q2.qz;

    double qx_new = q1.qw * q2.qx + q2.qw * q1.qx + q1.qy * q2.qz - q1.qz * q2.qy;
    double qy_new = q1.qw * q2.qy + q2.qw * q1.qy + q1.qz * q2.qx - q1.qx * q2.qz;
    double qz_new = q1.qw * q2.qz + q2.qw * q1.qz + q1.qx * q2.qy - q1.qy * q2.qx;

    return Quaternion(qw_new, qx_new, qy_new, qz_new);
}

// -------- Somega and Sq --------

Eigen::Matrix4d Somega(const Eigen::Vector3d& w)
{
    double wx = w(0);
    double wy = w(1);
    double wz = w(2);

    Eigen::Matrix4d S;
    S <<   0,   -wx,  -wy,  -wz,
           wx,    0,   wz,  -wy,
           wy,  -wz,    0,   wx,
           wz,   wy,  -wx,    0;

    return S;
}

Eigen::Matrix<double, 4, 3> Sq(const Eigen::Vector4d& q)
{
    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);

    Eigen::Matrix<double, 4, 3> S;
    S << -q1, -q2, -q3,
          q0, -q3,  q2,
          q3,  q0, -q1,
         -q2,  q1,  q0;
    return S;
}

Eigen::Matrix<double, 4, 3> Sq_q(const Quaternion& q)
{
    double q0 = q.qw;
    double q1 = q.qx;
    double q2 = q.qy;
    double q3 = q.qz;

    Eigen::Matrix<double, 4, 3> S;
    S << -q1, -q2, -q3,
          q0, -q3,  q2,
          q3,  q0, -q1,
         -q2,  q1,  q0;
    return S;
}

Eigen::Vector3d w_to_euler_rate(Eigen::Vector3d &w, Eigen::Vector3d &euler_angles)
{
    Eigen::Matrix3d T;
    Eigen::Vector3d euler_rates;
    float roll = euler_angles[0];
    float pitch = euler_angles[1];
    T << 1, std::sin(roll) * std::tan(pitch), std::cos(roll)*std::tan(pitch), 
         0, std::cos(roll), -std::sin(roll),
         0, std::sin(roll)/std::cos(pitch), std::cos(roll)/std::cos(pitch);

    euler_rates = T * w;

    return euler_rates;
  
}