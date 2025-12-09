#ifndef KGAIN_H
#define KGAIN_H
#include <..\..\.pio\libdeps\teensy41\Eigen\Dense>

Eigen::Vector3d U_K_att (double p_err, double q_err, double r_err, double roll_err, double pitch_err, double yaw_err);

Eigen::Vector3d U_K_I (double roll_err_int, double pitch_err_int, double yaw_err_int);

#endif