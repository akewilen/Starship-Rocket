#include <iostream>
#include <Eigen/Dense>
#include "Quaternion.h"   // contains Quaternion declaration
#include "Madgwick.h" //containts Madgwick declaration


int main() {
    // Just to see that the program actually runs
    std::cout << "Program started\n";

    // Initial quaternion (identity)
    Quaternion q0; // uses your default constructor

    // Some fake sensor data
    Eigen::Vector3d mag_meas(0.2, -0.1, 0.5);
    Eigen::Vector3d acc_meas(0.0, 0.0, 9.81);
    Eigen::Vector3d gyro_meas(0.01, -0.02, 0.005); // rad/s
    double gyro_error = 0.7923;
    double dt = 0.01;

    Quaternion q1 = madgwick_filter(
        acc_meas,
        gyro_meas,
        gyro_error,
        q0,
        dt
    );

    std::cout << "q1 = ["
              << q1.qw << ", "
              << q1.qx << ", "
              << q1.qy << ", "
              << q1.qz << "]\n";

    return 0;
}

//g++ madgwick_test.cpp Quaternion.cpp Madgwick.cpp \
 //   -I /opt/homebrew/opt/eigen/include/eigen3 \
  //  -std=c++17 -o app