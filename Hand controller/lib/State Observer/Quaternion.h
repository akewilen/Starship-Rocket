#pragma once
#include <Eigen/Dense>

class Quaternion {
public:
    double qw, qx, qy, qz;

    Quaternion();
    Quaternion(double w_, double x_, double y_, double z_);

    void Q_to_Euler(double &roll, double &pitch, double &yaw) const;
    Eigen::Matrix3d Q_to_Rotation() const;

    double norm() const;
    void normalize();

    Quaternion conjugate() const;
    Quaternion inverse() const;
    static Quaternion quat_mul(const Quaternion& q1, const Quaternion& q2);

    // Conversions
    Eigen::Vector4d q_to_double() const {
        return Eigen::Vector4d(qw, qx, qy, qz);
    }

    static Quaternion double_to_q(const Eigen::Vector4d &vec) {
        return Quaternion(vec[0], vec[1], vec[2], vec[3]);
    }
};

// 4x3 S(q) matrix from a quaternion
Eigen::Matrix<double, 4, 3> Sq_q(const Quaternion& q);

// 4x4 S(omega) matrix from a 3x1 angular velocity vector
Eigen::Matrix4d Somega(const Eigen::Vector3d& w);

// 4x3 S(q) matrix from a 4x1 quaternion vector
Eigen::Matrix<double, 4, 3> Sq(const Eigen::Vector4d& q);

// To get roll, pitch and yaw rates from angular velocity
Eigen::Vector3d w_to_euler_rate(Eigen::Vector3d &w, Eigen::Vector3d &euler_angles);