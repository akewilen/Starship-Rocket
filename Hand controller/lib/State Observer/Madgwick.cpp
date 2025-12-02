#include "Quaternion.h"
#include <Eigen/Dense>
#include <cmath>
#include "Madgwick.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix;
using Eigen::Matrix4d;

Quaternion madgwick_filter(
                           const Vector3d& acc_meas_in,
                           const Vector3d& gyro_meas,
                           double gyro_error,
                           const Quaternion& q_prev,
                           double dt)
{
    
    //Acc_meas_in: Accelerometer measurements (Acc - bias_acc)
    //gyro_meas: Gyroscope measurments (Acc - bias_gyro)
    //gyro error: Madgwick scalefactor, stationary gyro bias
    //q_prev: Quaternion from previous iteration
    //dt: Sample time

    Vector3d acc_meas = acc_meas_in;
    //Vector3d mag_meas = mag_meas_in;

    // ---------- Normalize accelerometer ----------
    acc_meas.normalize();
    double ax = acc_meas(0);
    double ay = acc_meas(1);
    double az = acc_meas(2);

    // Magnetometer components
    //double mx = mag_meas(0);
    //double my = mag_meas(1);
    //double mz = mag_meas(2);

    // Gyro components (rad/s)
    double wx = gyro_meas(0);
    double wy = gyro_meas(1);
    double wz = gyro_meas(2);

    // ---------- Current quaternion (from your class) ----------
    double qw = q_prev.qw;
    double qx = q_prev.qx;
    double qy = q_prev.qy;
    double qz = q_prev.qz;

    // Also as Eigen vector for matrix math
    Vector4d q_vec;
    q_vec << qw, qx, qy, qz;

    // ---------- Normalize magnetometer and form [0; m̂] as Quaternion ----------
   // Vector3d mag_norm = mag_meas.normalized();
   // Quaternion M(0.0,
   //              mag_norm(0),
   //              mag_norm(1),
   //              mag_norm(2));
//

    //Quaternion Qstar = q_prev.conjugate();


   //Quaternion Prod1 = Quaternion::quat_mul(q_prev, M);
    //Quaternion Hq    = Quaternion::quat_mul(Prod1, Qstar);

    // Convert h to Eigen vector for bx, bz computation
    //Vector4d h;
    //h << Hq.qw, Hq.qx, Hq.qy, Hq.qz;

    //double bx = std::sqrt(h(1)*h(1) + h(2)*h(2)); 
    //double bz = h(3);

    // ---------- Gravity error term F_g (3x1) ----------
    Vector3d F_g;
    F_g << 2*(qx*qz - qw*qy) - ax,
           2*(qw*qx + qy*qz) - ay,
           2*(0.5 - qx*qx - qy*qy) - az;

    // ---------- Jacobian J_g (3x4) ----------
    Matrix<double,3,4> J_g;
    J_g <<
        -2*qy,       2*qz,      -2*qw,      2*qx,
         2*qx,       2*qw,       2*qz,      2*qy,
             0,      -4*qx,      -4*qy,         0;



    Vector3d F_gb = F_g;
    Matrix<double,3,4> J_gb = J_g;

    // ---------- Skew-symmetric matrix S(omega)  ----------
    Matrix4d S_w = Somega(gyro_meas);  

    // ---------- Madgwick gain ----------
    double beta = std::sqrt(3.0/4.0) * std::abs(gyro_error);

    // ---------- Gradient term: grad = J^T * F ----------
    Vector4d grad = J_gb.transpose() * F_gb;
    double grad_norm = grad.norm();
    if (grad_norm > 0.0) {
        grad /= grad_norm;
    }

    // ---------- Quaternion derivative: q_dot = 0.5 * S_w * q ----------
    Vector4d q_dot = 0.5 * (S_w * q_vec);

    // ---------- Integrate and renormalize ----------
    Vector4d q_next_vec = q_vec + (q_dot - beta * grad) * dt;

    double qn_norm = q_next_vec.norm();
    if (qn_norm > 0.0) {
        q_next_vec /= qn_norm;
    }

    // Convert back to quaternion
    Quaternion q_next(q_next_vec(0), q_next_vec(1),
                      q_next_vec(2), q_next_vec(3));

    return q_next;
}
