function [x_updated, P_updated, x_pred, P_pred, w_eff] = EKF_with_bias(w, a, x, P, Rw , Rb, Ra, T, f)
% Extended Kalman filter for a quaternion motion model with gyro bias
% State: x = [ q ; b_g ] 
%   q   : 4x1 quaternion
%   b_g : 3x1 gyro bias (rad/s)

% w  : measured gyro (rad/s)  WARNING: RAW DATA IS DEG/S
% a  : accelerometer measurement (3x1)
% P  : 7x7 prior covariance
% Rw : 3x3 gyro noise covariance
% Rb : 3x3 gyro bias random-walk covariance
% Ra : 3x3 accel noise covariance
% T  : sampling time
% f  : known specific force (3x1), default [0;0;0], will need update if we
% are linearly accelerating

% ==================== OUTPUT NOTES ====================================
% w_eff: Gyroscope measurement corrected by estimated bias. Use this 
% instead of raw gyroscope measurements. Might be useful to apply a lowpass
% filter on it. 

%======================================================================

    if nargin < 9
        f = [0; 0; 0]; % Force acting on CM resulting in acceleration
    end

    % Extract quaternion and bias from state
    q  = x(1:4);
    bg = x(5:7);

    % ====== Prediction step ======

    % Use bias-compensated gyro
    w_eff = w - bg;

    % Nonlinear quaternion prediction 
    Fq = T/2 * Somega(w_eff) + eye(4);   % 4x4
    q_pred = Fq * q;
    q_pred = mu_normalizeQ(q_pred);      % Normalize quaternion

    % Bias is modeled as random walk (constant plus noise)
    bg_pred = bg;

    % Predicted state
    x_pred = [q_pred; bg_pred];

    % ----- Linearization for covariance prediction -----
    %
    % Continuous-time model:
    %   q_dot  = 1/2 * Sq(q) * (w - bg + n_w)
    %   bg_dot = n_b -> Bg_dot is assumed to be equal to the noise
    %
    % Discrete approximation:
    %   q_{k+1} = q_k + 1/2*T*Sq(q_k)*(w - bg) + T/2*Somega(n_w)q_k
    %   b_{k+1} = b_k + n_b
    % Jacobian wrt q:   Fq = I + 0.5*T*Somega(w_eff)
    % Jacobian wrt bg:  Fqb =-0.5*T * Sq(q)

    F = eye(7);
    F(1:4,1:4) = Fq;              % d q_{k+1} / d q_k
    F(1:4,5:7) = -T/2 * Sq(q);    % d q_{k+1} / d b_g

    % Process noise mapping:
    % noise vector: [ n_w ; n_b ] (6x1)
    %   n_w : gyro noise      (3x1)
    %   n_b : bias noise      (3x1)
    %
 

    G = zeros(7,6);
    G(1:4,1:3) = T/2 * Sq(q);   % effect of gyro noise on quaternion
    G(5:7,4:6) = eye(3);        % effect of bias noise on bias

    Q = blkdiag(Rw, Rb);        % 6x6 process noise covariance

    % Covariance prediction
    P_pred = F * P * F.' + G * Q * G.';
 
    % ====== EKF update step (accelerometer) ======

    g = [0.0131; -0.3506; 9.8356]; % Mean of accelerometer data when IMU stationary

    % Linearized accelerometer measurement model wrt quaternion
    Hq = Acc_measurment_model(q_pred, g, f);  % 3x4

    % Accel doesn't depend directly on gyro bias
    H = [Hq, zeros(3,3)];  % 3x7

    % Innovation covariance
    S = H * P_pred * H.' + Ra;

    % Kalman gain
    K = (P_pred * H.') *pinv(S);   

    % Nonlinear measurement prediction
    pred_rotation = Q_to_rotation(q_pred).'; 
    yacc_hat = pred_rotation * (g + f);  % 3x1

    % Innovation (measurement residual)
    innovation = a - yacc_hat;

    % State update
    x_updated = x_pred + K * innovation;

    % Covariance update
    P_updated = P_pred - K * S * K.';

    % Renormalize quaternion part only
    x_updated(1:4) = mu_normalizeQ(x_updated(1:4));

end
