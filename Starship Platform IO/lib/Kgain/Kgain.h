#ifndef KGAIN_H
#define KGAIN_H
#include <..\..\.pio\libdeps\teensy41\Eigen\Dense>

Eigen::Vector3d U_K_att (double p_err, double q_err, double r_err, double roll_err, double pitch_err, double yaw_err);

Eigen::Vector3d U_K_I (double roll_err_int, double pitch_err_int, double yaw_err_int);

void Integral_Error_Update_AntiWindup (double roll_err, double pitch_err, double yaw_err, double dt,
                                    double& roll_err_int, double& pitch_err_int, double& yaw_err_int,
                                    const Eigen::Vector3d& saturation_error,
                                    double& prev_roll_err, double& prev_pitch_err, double& prev_yaw_err,
                                    Eigen::Vector3d& prev_aw_term);

#endif