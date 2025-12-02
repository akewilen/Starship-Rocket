%% ============================================================
%  EKF vs EKF-with-bias Attitude Estimation Test Bench
%  - Truth: random-walk angular velocity
%  - Sensors: gyro (with drifting bias) + accel (gravity only)
%  - Filters:
%      1) EKF (no bias state, uses calibrated fixed gyro bias)
%      2) EKF_with_bias (state = [q; b], bias random walk), accelerometer
%      bias is removed in input.
%      3) Madgwick filter (Can choose wether or not to include
%      magnetometer)
% ============================================================

clear; clc; close all;

%% ============================================================
%  1) GLOBAL PARAMETERS & TUNING KNOBS
% ============================================================

% --- Simulation timing ---
dt      = 0.01;         % [s] sample time
Tend    = 50;           % [s] total time
N       = round(Tend/dt) + 1;
t       = (0:N-1) * dt;

% --- Gravity in world frame ---
g_world = [0; 0; 9.81]; % [m/s^2]

% --- True initial gyro bias (used in simulation) ---
bias_gyro_true0_deg = [-0.5607; -0.0342; -0.3706];       % [deg/s]
bias_gyro_true0     = deg2rad(bias_gyro_true0_deg);      % [rad/s]

% --- "Calibrated" gyro bias used by plain EKF (can be imperfect) ---
bias_gyro_calib_deg = [-0.5607; -0.0342; -0.3706];       % [deg/s]
bias_gyro_calib     = deg2rad(bias_gyro_calib_deg);      % [rad/s]

% --- Accelerometer bias (constant, for simulation) ---
bias_acc = [0.0277; 0.7364; 0.1344]; % [m/s^2]

% --- Gyro measurement noise covariance (per sample) ---
% Given covariance in (deg/s)^2, convert to (rad/s)^2
cov_gyro_deg = [ 0.0052    0.0001    0;
                     0.0001    0.0063   -0.0001;
                     0         -0.0001   0.0050 ];
cov_gyro_rad = cov_gyro_deg * (pi/180);     % [rad/s]

% --- Accelerometer measurement noise covariance (per sample) ---
% Given some base covariance in (m/s^2)^2, scaled down
cov_acc = [ 0.0283    0.0000   -0.0008;
              0.0000    0.0303   -0.0000;
             -0.0008   -0.0000    0.0212 ] / 1000;  % [ (m/s^2)^2 ]

% --- True gyro bias drift (for simulation) ---
b_drift_deg = 0.01;                           % [deg/s] per sqrt(sec)
b_drift_rad     = deg2rad(b_drift_deg);     % [rad/s] per sqrt(sec)

% --- EKF process noise tuning ---
% These are DISCRETE process noise covariances per timestep
Qw  = 1e-4 * eye(3);                      % gyro process noise (model)
Qw = cov_gyro_rad;
Qb = 1e-1 * (b_drift_rad ^2 * dt) * eye(3);    % bias random-walk noise
                                                   

% --- EKF measurement noise tuning (accelerometer) ---
Ra =  (1.0e-3 * [ 0.2831    0.0004   -0.0075;
                               0.0004    0.3034   -0.0001;
                              -0.0075   -0.0001    0.2125 ]);  % [ (m/s^2)^2 ]

% --- EKF initial covariances ---
P0_q     = eye(4);                        % initial quaternion covariance
sigma_b0 = deg2rad(5);                    % 5 deg/s initial bias uncertainty
P0_b     = (sigma_b0^2) * eye(3);        % variance!
P0_prior = blkdiag(P0_q, P0_b);          % for EKF_with_bias
P0       = P0_q;                         % for plain EKF

% --- Initial states ---
q0        = [1; 0; 0; 0];                % initial attitude (unit quaternion)
x_prior   = [q0; bias_gyro_calib];       % EKF_with_bias initial state
q0_bias   = q0;                          % same initial attitude
q0_madgwick = q0; 



%% ============================================================
%  2) SIMULATE TRUE MOTION (ANGULAR VELOCITY & ATTITUDE)
% ============================================================

omega_true = zeros(3, N);                % true angular velocity [rad/s]
omega_true(:,1) = deg2rad([2; 1; 0.5]);  % small initial rate

for k = 2:N
    % Random walk in angular velocity
    omega_true(:,k) = omega_true(:,k-1) + deg2rad(5 * dt) * randn(3,1);

    % Optionally limit max rate
    omega_true(:,k) = max(min(omega_true(:,k), deg2rad(50)), -deg2rad(50));
end

% Integrate quaternion to get true attitude
q_true      = zeros(4,N);
q_true(:,1) = q0;

for k = 2:N
    q_true(:,k) = quat_propagate(q_true(:,k-1), omega_true(:,k), dt);
end

% True Euler angles (for plotting)
euler_true = zeros(3,N);
for k = 1:N
    [roll, pitch, yaw] = Q_to_Euler(q_true(:,k));
    euler_true(:,k) = [roll; pitch; yaw];
end



%% ============================================================
%  3) SIMULATE "TRUE" ACCELERATION (ONLY GRAVITY, NO LINEAR ACCEL)
% ============================================================

acc_true = zeros(3,N);

for k = 1:N
    R = Q_to_rotation(q_true(:,k));  % rotation matrix from body to world or vice versa,
                                     % assumed body->world here
    % We want acceleration in body frame: a_body = R^T * g_world
    acc_true(:,k) = R.' * g_world;
end



%% ============================================================
%  4) SIMULATE TRUE GYRO BIAS DRIFT
% ============================================================

b_true = zeros(3, N);
b_true(:,1) = bias_gyro_true0;  % initial true bias

for k = 2:N
    % Random walk for true bias: b_{k+1} = b_k + sigma_b_drift * sqrt(dt) * w
    b_true(:,k) = b_true(:,k-1) + b_drift_rad  * sqrt(dt) * randn(3,1);
end



%% ============================================================
%  5) SIMULATE NOISY IMU MEASUREMENTS
% ============================================================

gyro_meas = zeros(3,N);
acc_meas  = zeros(3,N);
mag_meas  = ones(3,N); %Dont care about this currently
for k = 1:N
    % Gyro: true omega + drifting bias + noise
    gyro_meas(:,k) = omega_true(:,k) + b_true(:,k) +  mvnrnd(zeros(3,1), cov_gyro_rad, 1).';

    % Accel: true gravity + bias + noise
    acc_meas(:,k)  = acc_true(:,k)   + bias_acc +  mvnrnd(zeros(3,1), cov_acc, 1).';
end

% For visualization only: "measured orientation" (noisy)
euler_meas = euler_true + deg2rad(0.5) * randn(3,N);



%% ============================================================
%  6) QUICK PLOT: TRUE vs NOISY ORIENTATION (FOR REFERENCE)
% ============================================================

euler_true_deg = rad2deg(euler_true);
euler_meas_deg = rad2deg(euler_meas);

figure;
subplot(3,1,1);
plot(t, euler_true_deg(1,:), 'k', t, euler_meas_deg(1,:), 'r');
ylabel('Roll [deg]'); legend('true','meas'); title('Synthetic trajectory');

subplot(3,1,2);
plot(t, euler_true_deg(2,:), 'k', t, euler_meas_deg(2,:), 'r');
ylabel('Pitch [deg]'); legend('true','meas');

subplot(3,1,3);
plot(t, euler_true_deg(3,:), 'k', t, euler_meas_deg(3,:), 'r');
ylabel('Yaw [deg]'); xlabel('Time [s]'); legend('true','meas');



%% ============================================================
%  7) EKF & MADGEWICK SETUP (PLAIN EKF & EKF WITH BIAS)
% ============================================================

% Covariances for filters
Rw = Qw;      % process noise (gyro part)
Rb = Qb;      % process noise (bias random walk)
Ra = Ra;       % accel measurement noise

% States & logs
q_mat       = zeros(4, N);  % plain EKF quat
q_mat_bias  = zeros(4, N);  % EKF-with-bias quat
q_mat_madgewick = zeros(4, N); %Madgewick

q_mat(:,1)      = q0;
q_mat_bias(:,1) = q0_bias;
q_mat_madgewick(:,1) = q0_madgwick;

euler_angles       = zeros(3, N);  % plain EKF euler
euler_angles_bias  = zeros(3, N);  % bias EKF euler
euler_angles_madgewick = zeros(3, N); % Madgewick filter
euler_angles(:, 1)      = Q_to_Euler(q0);
euler_angles_bias(:, 1) = Q_to_Euler(q0_bias);
euler_angles_madgewick(:, 1) = Q_to_Euler(q0_madgwick);

w_eff = zeros(3, N); %Bias corrected gyroscope from EKF with bias
% For EKF_with_bias, keep covariance separate
P0_plain  = P0;        % 4x4
P0_biasKF = P0_prior;  % 7x7

bias_state_vec = zeros(3,N); %Storing bias estimate from bias EKF


%% ============================================================
%  8) MAIN FILTER LOOP
% ============================================================

for k = 2:N

    % ---------- Plain EKF (known constant bias) ----------
    gyro_in_plain = gyro_meas(:,k) - bias_gyro_calib;  % compensate with calibration
    acc_in_plain  = acc_meas(:,k)  - bias_acc;         % accel bias known
    mag_in_plain = mag_meas(:,k); % Magnetometer currently ignored

    [x_updated, P_updated, ~, ~] = EKF( ...
        gyro_in_plain, acc_in_plain, ...
        q0, P0_plain, Rw, Ra, dt);

    q_mat(:,k) = x_updated;
    q0         = x_updated;
    P0_plain   = P_updated;

    [roll, pitch, yaw] = Q_to_Euler(q0);
    euler_angles(:,k)  = [roll; pitch; yaw];


    % ---------- EKF WITH BIAS IN STATE ----------
    % NOTE: EKF_with_bias should internally do w_eff = w - bg 
    gyro_in_bias = gyro_meas(:,k);  % raw, biased gyro
    acc_in_bias  = acc_meas(:,k) - bias_acc;   % raw accel (with bias)

    [x_updated_bias, P_updated_bias, ~, ~, w_eff(:,k)] = EKF_with_bias( ...
        gyro_in_bias, acc_in_bias, ...
        x_prior, P0_biasKF, Rw, Rb, Ra, dt);
    
    q_mat_bias(:,k) = x_updated_bias(1:4);
    x_prior         = x_updated_bias;
    P0_biasKF       = P_updated_bias;
    
    bias_state_vec(:, k) = x_updated_bias(5:end);

    [roll_b, pitch_b, yaw_b] = Q_to_Euler(q_mat_bias(:,k));
    euler_angles_bias(:,k)   = [roll_b; pitch_b; yaw_b];

    % % ---------- Madgewick Filter  ----------
    q_mat_madgewick(:, k) = madgewick_filter( acc_in_plain ,  gyro_in_plain, bias_gyro_calib,  q_mat_madgewick(:, k-1), dt);
    [roll_madge, pitch_madge, yaw_madge] = Q_to_Euler(q_mat_madgewick(:, k));
    euler_angles_madgewick(:, k) = [roll_madge, pitch_madge, yaw_madge].';

end

% Unwrap and convert to degrees
euler_rad          = euler_angles;
euler_rad_unwrap   = unwrap(euler_rad.').';
euler_deg          = rad2deg(euler_rad_unwrap);

euler_rad_bias         = euler_angles_bias;
euler_rad_unwrap_bias  = unwrap(euler_rad_bias.').';
euler_deg_bias         = rad2deg(euler_rad_unwrap_bias);

euler_rad_madgewick =          euler_angles_madgewick;
euler_rad_unwrap_madgewick =   unwrap(euler_rad_madgewick.').';
euler_deg_madgewick =          rad2deg(euler_rad_unwrap_madgewick);



%% ============================================================
%  9) PLOTS: TRUE vs EKF vs EKF-with-bias
% ============================================================

figure;
subplot(3,1,1);
plot(t, rad2deg(unwrap(euler_true(1,:))), 'k', 'LineWidth', 1);
hold on;
plot(t, euler_deg(1,:), 'b', 'LineWidth', 1);
plot(t, euler_deg_bias(1,:), 'r', 'LineWidth', 1);
plot(t, euler_deg_madgewick(1,:), 'LineWidth', 1)
ylabel('Roll [deg]');
xlabel('Time [s]');
legend('True Trajectory','EKF','EKF with bias estimation', 'Madgewick');

subplot(3,1,2);
plot(t, rad2deg(unwrap(euler_true(2,:))), 'k');
hold on;
plot(t, euler_deg(2,:), 'b', 'LineWidth', 1);
plot(t, euler_deg_bias(2,:), 'r', 'LineWidth', 1);
plot(t, euler_deg_madgewick(2,:), 'LineWidth', 1)
ylabel('Pitch [deg]');
xlabel('Time [s]');
legend('True Trajectory','EKF','EKF with bias estimation', 'Madgewick');

subplot(3,1,3);
plot(t, rad2deg(unwrap(euler_true(3,:))), 'k');
hold on;
plot(t, euler_deg(3,:), 'b', 'LineWidth', 1);
plot(t, euler_deg_bias(3,:), 'r', 'LineWidth', 1);
plot(t, euler_deg_madgewick(2,:), 'LineWidth', 1)
ylabel('Yaw [deg]');
xlabel('Time [s]');
legend('True Trajectory','EKF','EKF with bias estimation', 'Madgewick');


%%

figure 
subplot(2,1,1)
plot(t,rad2deg(b_true(1,:)), 'LineWidth', 1.5)
hold on
plot(t,rad2deg(bias_state_vec(1,:)), 'LineWidth', 1.5)
legend("Bx_{true}", "\hat{B_x}")
ylabel("Bias [deg]")
xlabel("Time [s]")
subplot(2,1,2)
plot(t,rad2deg(b_true(2,:)), 'LineWidth', 1.5)
hold on
plot(t,rad2deg(bias_state_vec(2,:)), 'LineWidth', 1.5)
ylabel("Bias [deg]")
xlabel("Time [s]")
legend("By_{true}", "\hat{B_y}")
title("Bias estimation versus true bias")

%% Mean squared error of the filters
%MSE_EKF = sum((q_true(:) - q_mat(:)).^2)/N
%MSE_EKF_bias = sum((q_true(:) - q_mat_bias(:)).^2)/N
%MSE_madgewick = sum((q_true(:) - q_mat_madgewick(:)).^2)/N


true_roll_pitch = rad2deg(euler_true(1:2, :));
EKF_roll_pitch = euler_deg(1:2,:);
EKF_bias_roll_pitch = euler_deg_bias(1:2,:);
madgewick_roll_pitch = euler_deg_madgewick(1:2,:);

MSE_EKF = sum((true_roll_pitch(:) - EKF_roll_pitch(:)).^2)/N
MSE_EKF_bias =  sum((true_roll_pitch(:) - EKF_bias_roll_pitch(:)).^2)/N
MSE_madgewick = sum((true_roll_pitch(:) - madgewick_roll_pitch(:)).^2)/N
