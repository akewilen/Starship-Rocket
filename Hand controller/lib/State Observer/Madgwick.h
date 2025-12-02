#pragma once
#include <Eigen/Dense>

// mag_meas, acc_meas, gyro_meas: 3x1 vectors
// q: 4x1 unit quaternion [qw qx qy qz]
// gyro_error: mean gyro error (scalar)
// dt: time step
//Add const Eigen::Vector3d& mag_meas as argument for mag
Quaternion madgwick_filter(
                                const Eigen::Vector3d& acc_meas,
                                const Eigen::Vector3d& gyro_meas,
                                double gyro_error,
                                const Quaternion& q,
                                double dt);
