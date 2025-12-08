#include "Quaternion.h"
#include <Eigen/Dense>
#include <iostream>

Eigen::Matrix<double, 3, 4> Acc_measurement_model(
    const Quaternion &x,
    const Eigen::Vector3d& acc_mean, 
    const Eigen::Vector3d& f)
 {
    double qw = x.qw; double qx = x.qx; double qy = x.qy; double qz = x.qz ; //State prediction (quaternion)
    double acc_mean_x = acc_mean(0); double acc_mean_y = acc_mean(1); double acc_mean_z = acc_mean(2);   //Mean of accelerometer data when IMU stationary
    double fx = f[0]; double fy = f[1]; double fz = f[2]; //Linear accleration

    Eigen::Matrix<double, 3, 4> H;

    H <<
    // Row 1
    4 * qw * (fx + acc_mean_x) + 2 * qz * (fy + acc_mean_y) - 2 * qy * (fz + acc_mean_z),
    4 * qx * (fx + acc_mean_x) + 2 * qy * (fy + acc_mean_y) + 2 * qz * (fz + acc_mean_z),
    2 * qx * (fy + acc_mean_y) - 2 * qw * (fz + acc_mean_z),
    2 * qw * (fy + acc_mean_y) + 2 * qx * (fz + acc_mean_z),

    // Row 2
    4 * qw * (fy + acc_mean_y) - 2 * qz * (fx + acc_mean_x) + 2 * qx * (fz + acc_mean_z),
    2 * qy * (fx + acc_mean_x) + 2 * qw * (fz + acc_mean_z),
    2 * qx * (fx + acc_mean_x) + 4 * qy * (fy + acc_mean_y) + 2 * qz * (fz + acc_mean_z),
    2 * qy * (fz + acc_mean_z) - 2 * qw * (fx + acc_mean_x),

    // Row 3
    2 * qy * (fx + acc_mean_x) - 2 * qx * (fy + acc_mean_y) + 4 * qw * (fz + acc_mean_z),
    2 * qz * (fx + acc_mean_x) - 2 * qw * (fy + acc_mean_y),
    2 * qw * (fx + acc_mean_x) + 2 * qz * (fy + acc_mean_y),
    2 * qx * (fx + acc_mean_x) + 2 * qy * (fy + acc_mean_y) + 4 * qz * (fz + acc_mean_z);
    return H;
 }

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
              Eigen::Vector3d f)
 {  
    // ==================== INPUTS ==============================
    //w: Gyroscope measurement [rad/s]
    //a: Accelermoter measurement 
    //x: Prior quaternion as a 4d vector
    //P: Prior covariance matrix
    //Rw: Gyroscope covariance matrix in [rad/s], see testEKF.cpp for actual value
    //Ra: Accelerometer covariance matrix
    //T: Sample time
    //f: Linear acceleration of the COM
    // ===========================================================

    // ==================== OUTPUT ==============================
    // Result is a struct with 2 parameters containing:
    // 1. Updated state
    // 2. Posterior Covariance


    EKFResult Result;

    Eigen::Matrix4d F,  P_pred;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    F = T/2.0 * Somega(w) + I;
    Quaternion x_pred = Quaternion::double_to_q(F * x);
    x_pred.normalize();
    Eigen::Vector4d x_pred_vec = x_pred.q_to_double();
    //F = @(w, T) T/2 * Somega(w) + eye(4); % Linearized quaternion motion model
    //G = @(x)  T/2 * Sq(x); %Noise part of linearized motion model
    

    Eigen::Matrix <double, 4, 3> G = T/2.0 * Sq(x_pred_vec);
    P_pred = F * P * F.transpose() + G * Rw * G.transpose();
    Eigen::Vector3d acc_mean(0.0, 0.0, 9.82);   //Can calculate this in setup and have as argument instead (acc_mean - acc_bias)
   

    Eigen::Matrix <double, 3,4> H = Acc_measurement_model(x_pred, acc_mean, f); //Linearized measurement model

    Eigen::Matrix3d S = H * P_pred * H.transpose() + Ra; //Measurement model uncertainty propagation
    Eigen::Matrix <double, 4,3> K = (P_pred * H.transpose()) * S.inverse(); //Kalman gain

    Eigen::Matrix3d Predicted_rotation = x_pred.Q_to_Rotation().transpose();
    Eigen::Vector3d y_hat = Predicted_rotation * (acc_mean + f); //Nonlinear measurement model
    Eigen::Vector3d Innovation = a - y_hat; //Kalman innovation
   // std::cout << "Innovation: " << Innovation.transpose() << "\n";
    Eigen::Vector4d x_updated = x_pred_vec + K*Innovation; //Corrected state
    Quaternion x_updated_q = Quaternion::double_to_q(x_updated); //convert to quaternion
    x_updated_q.normalize();

    Eigen::Matrix4d P_updated = P_pred - K*S*K.transpose(); //Corrected uncertainty matrix

    Eigen::Vector4d x_updated_vec = x_updated_q.q_to_double(); //Corrected state as a 4d vector for easier handling outside of function.
   
    Result.x_updated = x_updated_vec;
    Result.P_updated = P_updated;

    return Result;
 }             


