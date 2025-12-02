#pragma once
#include <Eigen/Dense>
#include "Quaternion.h"
//3x4
Eigen::Matrix<double, 3, 4> Acc_measurement_model(
    const Quaternion& x, 
    const Eigen::Vector3d& acc_mean, 
    const Eigen::Vector3d& f);

 struct EKFResult { //In order to store the results from the EKF
    Eigen::Vector4d x_updated;
    Eigen::Matrix4d P_updated;

};
EKFResult EKF(Eigen::Vector3d w,
                 Eigen::Vector3d a, 
              Eigen::Vector4d x, 
              Eigen::Matrix4d P, 
              Eigen::Matrix3d Rw,
              Eigen::Matrix3d Ra,
              float T, 
              Eigen::Vector3d f);
 

