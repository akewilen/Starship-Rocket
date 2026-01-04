
#include "Quaternion.h"
#include <Eigen/Dense>

struct AltitudeResult{
    Eigen::Vector4d x_updated;
    Eigen::Matrix4d P_updated;
};


AltitudeResult Altitude_KF(
    const Quaternion& q,
    const Eigen::Vector3d& acc,
    const Eigen::Vector4d& x,
    const Eigen::Matrix4d& P,
    double T,
    const Eigen::Matrix3d& Q,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& meas);

