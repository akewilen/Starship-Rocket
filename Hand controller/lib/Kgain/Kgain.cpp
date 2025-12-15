
#include <..\..\.pio\libdeps\teensy41\Eigen\Dense>

Eigen::Matrix <double, 3, 6> K_attitude {
    {4.6015,    0.0000,    0.0000,   46.9156,    0.0000,   -0.0000},
    {-0.0000,    4.7123,    0.0000,   -0.0000,   46.9156,   -0.0000},
    {-0.0000,   -0.0000,   31.6736,   -0.0000,   -0.0000,    0.0995}
    };

Eigen::Matrix<double, 3, 1> K_I {
    {0.0000},
    {0.0000},
    {9.9504}
    };

Eigen::Vector3d U_K_att (double p_err, double q_err, double r_err, double roll_err, double pitch_err, double yaw_err) {

    Eigen::Matrix<double, 6, 1> state_err;
    state_err << p_err, q_err, r_err, roll_err, pitch_err, yaw_err;
    Eigen::Vector3d u_k = -K_attitude * state_err;

    return u_k;
}

Eigen::Vector3d U_K_I (double roll_err_int, double pitch_err_int, double yaw_err_int) {

    Eigen::Matrix<double, 3, 1> state_err_int;
    state_err_int << roll_err_int, pitch_err_int, yaw_err_int;
    Eigen::Vector3d u_k_i = K_I.cwiseProduct(state_err_int);

    return u_k_i;
}