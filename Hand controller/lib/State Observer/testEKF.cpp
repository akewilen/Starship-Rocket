#include <iostream>
#include <Eigen/Dense>

#include "Quaternion.h"
#include "EKF.h"   // put your EKF function + EKFResult struct here


Eigen::Vector3d gyro_mean(-0.0123, -0.0026, 0.0058); // [rad/s]
Eigen::Vector3d acc_mean(0.0378, 0.0013, 10.0035);



int main()
{
    // --- IMU test inputs ---
    Eigen::Vector3d w;   // angular velocity (rad/s)
    Eigen::Vector3d a;   // accelerometer measurement (m/s^2)
    Eigen::Vector3d f;   // body-frame gravity model input

    w << 0.01, -0.02, 0.005;        // small rotation rates
    a << 0.0, 0.0, 9.81;            // stationary, measuring gravity
    f << 0.0, 0.0, 0.0;           //  linear acc;

    // --- Initial state quaternion (no rotation) ---
    Eigen::Vector4d x0;
    x0 << 1, 0, 0, 0;               // identity quaternion

    // --- Initial covariance ---
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() ;

    // --- Process noise (gyro noise covariance) ---
    Eigen::Matrix3d Rw, Rw_unscaled;
    Rw_unscaled << 0.0056,    0.0004 ,   0.0001,
                   0.0004,    0.0063 ,   0.0002,
                   0.0001,    0.0002 ,   0.0047;
    Rw = Rw_unscaled * M_PI/180; //Convert Rw to Rad/s

    // --- Measurement noise (accelerometer noise covariance) ---
    Eigen::Matrix3d Ra, Ra_unscaled;
    Ra_unscaled << 0.2596, 0.0135, 0.0025,
        0.0135, 0.2925, -0.0086,
        0.0025, -0.0086, 0.1750;
    Ra = Ra_unscaled * 1e-3;

    
    // --- timestep ---
    float T = 0.01f;                // 10 ms

    // Call the EKF
    EKFResult res = EKF(w, a, x0, P0, Rw, Ra, T, f);

    // --- Print results ---
    std::cout << "Updated quaternion state: " << std::endl
              << res.x_updated.transpose() << std::endl << std::endl;

    std::cout << "Updated covariance: " << std::endl
              << res.P_updated << std::endl;

    return 0;
}
