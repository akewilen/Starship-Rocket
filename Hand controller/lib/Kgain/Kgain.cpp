
#include <..\..\.pio\libdeps\teensy41\Eigen\Dense>

Eigen::Matrix <double, 3, 6> K_attitude {
    { 13.7800,   -0.0000,    0.0000,   65.2160,   -0.0000,   -0.0000},
    { -0.0000,   13.7800,   -0.0000,   -0.0000,   65.2160,   -0.0001},
    {  0.0000,   -0.0000,    9.5234,    0.0000,   -0.0000,    0.3930}
};
      
Eigen::Matrix<double, 3, 1> K_I {
    {0.01},
    {0.01},
    {9.9504}
    };

Eigen::Vector3d Kaw = Eigen::Vector3d::Constant(2.0);

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

void Integral_Error_Update_AntiWindup (double roll_err, double pitch_err, double yaw_err, double dt,
                                    double& roll_err_int, double& pitch_err_int, double& yaw_err_int,
                                    const Eigen::Vector3d& saturation_error,
                                    double& prev_roll_err, double& prev_pitch_err, double& prev_yaw_err,
                                    Eigen::Vector3d& prev_aw_term) {
    
    // Calculate anti-windup terms
    Eigen::Vector3d aw_term = Kaw.cwiseProduct(saturation_error);
    
    // Trapezoidal integration of (error + anti_windup_term)
    roll_err_int += (dt / 2.0) * ((roll_err + aw_term(0)) + (prev_roll_err + prev_aw_term(0)));
    pitch_err_int += (dt / 2.0) * ((pitch_err + aw_term(1)) + (prev_pitch_err + prev_aw_term(1)));
    yaw_err_int += (dt / 2.0) * ((yaw_err + aw_term(2)) + (prev_yaw_err + prev_aw_term(2)));
    
    // Save values for next iteration
    prev_roll_err = roll_err;
    prev_pitch_err = pitch_err;
    prev_yaw_err = yaw_err;
    prev_aw_term = aw_term;
}