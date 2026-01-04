
#include "Quaternion.h"
#include <Eigen/Dense>
#include <iostream>

AltitudeResult Altitude_KF(
    const Quaternion& q,
    const Eigen::Vector3d& acc,
    const Eigen::Vector4d& x,
    const Eigen::Matrix4d& P,
    double T,
    const Eigen::Matrix3d& Q,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& meas
    ) 
{

    //Measurement: meas = [vx_meas; vy_meas; z_meas]
    //R: blkdiag([var(vx), var(vy), var(z)]), where vx and vy is from optical flow and z from lidar. 
  
    AltitudeResult result; 
    Eigen::Vector3d gravity(0.0, 0.0, 9.82); 
    double scale_factor = 0.001833; //Optical flow scaling parameter
    double roll; double pitch; double yaw;
    
    q.Q_to_Euler(roll, pitch, yaw) //Extract euler angles from quaternion 

    Eigen::Matrix3d body_to_world = q.Q_to_Rotation(); //Converts quaternion to a rotation matrix
    Eigen::Vector3d a_world = body_to_world * acc - gravity; // Converts accelerometer readings to world coordinate frame, remove gravity to only get linear acceleration
    Eigen::Vector3d meas_to_world = body_to_world * Eigen::Vector3d(0, 0, meas(2)); //Want measurements in world coordinate frame, in particular the lidar measurement
    double lidar_z = meas_to_world(2);
    double vx = x(0); double vy = x(1); double vz = x(2); double z = x(3);

    Eigen::Vector4d x_pred;
    x_pred(0) = vx + T *  a_world(0);
    x_pred(1) = vy + T * a_world(1);
    x_pred(2) = vz + T * a_world(2);
    x_pred(3) = z + T * vz + 0.5*T*T * a_world(2);

  

    Eigen::Matrix4d A;
    A << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, T, 1;

    double qw = q.qw; double qx = q.qx; double qy = q.qy; double qz = q.qz;

    //Eigen::Matrix<double, 4, 3> B;
    //B << 
    ////Row 1
    //T*(2*qw*qw + 2*qx*qx - 1), -T*(2*qw*qz - 2*qx* qy), T*(2*qw*qy + 2*qx*qz),
    ////Row 2
    //T*(2*qw*qz + 2*qx*qy), T*(2*qw*qw + 2*qy*qy - 1), -T*(2*qw*qx - 2*qy*qz),
    ////Row 3
    //-T*(2*qw*qy - 2*qx*qz), T*(2*qw*qx + 2*qy*qz), T*(2*qw*qw + 2*qz*qz - 1),
    ////Row 4
    //-(T*T*(2*qw*qy - 2*qx*qz))*0.5, (T*T*(2*qw*qx + 2*qy*qz))*0.5, (T*T*(2*qw*qw + 2*qz*qz - 1))*0.5;

    Eigen::Matrix<double, 4, 3> B;
    B.block<3,3>(0,0) = T * body_to_world;
    B.row(3)          = 0.5 * T * T * body_to_world.row(2);

    Eigen::Matrix4d P_pred = A*P*A.transpose() + B*Q*B.transpose(); //Uncertainty in prediction


    Eigen::Matrix<double, 3,4> H;      // Measurement model H
    H << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
    
    Eigen::Vector3d y;

    //Convert sensor readings to desired measurements. This results in a consistant innovation.
    y(2) = lidar_z;
    y(0) = k * lidar_z * meas(0);
    y(1) = k * lidar_z * meas(1);

    Eigen::Vector3d innovation = y - H*x_pred; //Kalman innovation
    Eigen::Matrix3d S = H * P_pred * H.transpose() + R; //Innovation covariance
    
    Eigen::Matrix<double, 4, 3> K = P_pred * H.transpose() * S.inverse(); //Kalman gain

    //update step
    Eigen::Vector4d x_updated = x_pred + K*innovation; //Corrected state
    Eigen::Matrix4d P_updated = P_pred - K*S*K.transpose();

    result.x_updated = x_updated;
    result.P_updated = P_updated;

    return result;

}

