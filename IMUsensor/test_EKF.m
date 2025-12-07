%% Testing of EKF

filename = "madge_test.csv";
%filename = "IMU_global_vs_local.csv";
%filename = "IMUdata.csv";
%filename = "more_data.csv";

data = csvread(filename);

N = size(data, 1);

acc = data(:, 1:3).'; % Accelerometer readings
gyro = data(:, 4:6).';% Gyroscope readings
mag = data(:, 7:end).'; %Magnetometer readings

bias_acc = [0.0277 0.7364 0.1344].';
bias_gyro = [-0.5607 -0.0342 -0.3706].';
bias_mag = [83.3796  220.8556  217.8227].'; 
% Correct sensor readings
acc_corr = acc - bias_acc ;
gyro_corr = deg2rad(gyro - bias_gyro) ;
mag_corr = mag - bias_mag  ;

gyro_corr_bias_EKF = deg2rad(gyro - [0;0;bias_gyro(3)]);

%Q = eye(4)* 1e-8; %Process noise
T = 0.01;
P0 = eye(4);
q0 = [1 0 0 0].';
q0_bias = q0;

sigma_b0 = deg2rad(5);  %5degree/s uncertainty intially in drift
P0_prior = blkdiag(eye(4), eye(3)*sigma_b0^2);

x_prior = [q0; zeros(3,1)];

Rw = [0.0052    0.0001    0.0000;
    0.0001    0.0063   -0.0001;
    0.0000   -0.0001    0.0050];

Rw = Rw * (pi/180)^2;

Ra = 1.0e-03 *[ 0.2831    0.0004   -0.0075;
    0.0004    0.3034   -0.0001;
   -0.0075   -0.0001    0.2125]; 

%Rb and Rw is process noise Q matrix
%Rw = 1e-4*eye(3);
%Rb = deg2rad(eye(3) * 0.0117^2); %Gyro bias
Rb = 1e-1 * (b_drift_rad^2 * dt) * eye(3); 
%Rb = 1e-1*eye(3)


%Ra = Ra * 100;

q_mat = zeros(4, N);
q_mat_bias = zeros(4, N);
q_mat(:, 1) = q0;
q_mat_bias(:, 1) = q0;
dt = T;
sim_time = 0:dt:dt*N-dt; %Simulation time

euler_angles = zeros(3, N);
euler_angles(:, 1) = Q_to_Euler(q0);

euler_angles_bias = zeros(3, N);
euler_angles_bias(:, 1) = Q_to_Euler(q0_bias);

for k = 2:N
    [x_updated, P_updated, x_pred, P_pred] = EKF(gyro_corr(:, k), acc_corr(:, k), q0, P0, Rw, Ra, T);
    [x_updated_bias, P_updated_bias, x_pred_bias, P_pred_bias] = EKF_with_bias(deg2rad(gyro(:,k)), acc_corr(:, k), x_prior, P0_prior, Rw, Rb, Ra, T);
 
 
    q_mat(:, k) = x_updated;
    q_mat_bias(:,k) = x_updated_bias(1:4);
    q0 = x_updated;
    P0 = P_updated;
    x_prior = x_updated_bias;
    P0_prior = P_updated_bias;
    [roll, pitch, yaw] = Q_to_Euler(q0);
    [roll_bias, pitch_bias, yaw_bias] = Q_to_Euler(x_prior(1:4));
    euler_angles(:, k) = [roll pitch yaw].';
    euler_angles_bias(:, k) = [roll_bias pitch_bias yaw_bias].';

end

euler_rad = euler_angles;                 
euler_rad_unwrapped = unwrap(euler_rad.').';  
euler_deg = rad2deg(euler_rad_unwrapped);

euler_rad_bias = euler_angles_bias;                 
euler_rad_unwrapped_bias = unwrap(euler_rad_bias.').';  
euler_deg_bias = rad2deg(euler_rad_unwrapped_bias);

%%
acc_euler = zeros(3,N);

for k = 1:N
    ax = acc_corr(1,k);
    ay = acc_corr(2,k);
    az = acc_corr(3,k);

    roll_acc  = atan2( ay,  az);                     % φ
    pitch_acc = atan2(-ax, sqrt(ay^2 + az^2));       % θ
    yaw_acc   = 0;  % or keep as NaN if you want to signal "unknown"

    acc_euler(:,k) = [roll_acc; pitch_acc; yaw_acc];
end

acc_euler_deg = rad2deg(unwrap(acc_euler.').');


%% Euler angle approximation from quaternion integration
%q = [1; 0; 0; 0];     % initial orientation
dt = 0.01;

% Calculate initial orientation from accelerometer reading
a0 = acc_corr(:,1);
ax = a0(1); ay = a0(2); az = a0(3);

roll  = atan2( ay,  az);                         % φ
pitch = atan2(-ax, sqrt(ay^2 + az^2));           % θ
yaw   = 0;                                       % ψ (unknown from accel)

q = Euler_to_Q(roll, pitch, yaw);
q = [q.w; q.x; q.y; q.z];
q = mu_normalizeQ(q);
%q = [1; 0; 0; 0];
q_hist = zeros(4, N);
q_hist(:,1) = q;

integrated_euler_angles = zeros(3, N);
integrated_euler_angles(:, 1) = Q_to_Euler(q);
for k = 2:N
    omega_k = gyro_corr(:,k);        % [wx; wy; wz] at time k
    q = quat_propagate(q, omega_k, dt);
    [roll, pitch, yaw] = Q_to_Euler(q);
    integrated_euler_angles(:, k) = [roll, pitch, yaw].';
    q_hist(:,k) = q;
end

int_euler_rad = integrated_euler_angles; 
int_euler_rad_unwrapped = unwrap(int_euler_rad.').';
int_euler_deg = rad2deg(int_euler_rad_unwrapped);


%%
figure
subplot(3,1,1);
plot(sim_time, euler_deg(1,:))
hold on
plot(sim_time, int_euler_deg(1,:))
xlabel("Time [s]")
ylabel("Roll [d]")
legend("EKF", "Gyroscope")
title('EKF vs numerical Gyroscope estimation of \phi');

subplot(3,1,2);
plot(sim_time, euler_deg(2,:))
hold on
plot(sim_time,  int_euler_deg(2,:))
xlabel("Time [s]")
ylabel("Pitch [d]")
title('EKF vs numerical Gyroscope estimation of \theta');
legend("EKF", "Gyroscope")

subplot(3,1,3);
plot(sim_time, euler_deg(3,:))
hold on
plot(sim_time,  int_euler_deg(3,:))
xlabel("Time [s]")
ylabel("Yaw [d]")
title('EKF vs numerical Gyroscope estimation of \psi');
legend("EKF", "Gyroscope")
%%
%%
figure
subplot(3,1,1);
plot(sim_time, euler_deg(1,:), "LineWidth",1)
hold on
plot(sim_time, euler_deg_bias(1,:), "LineWidth",1)
hold on
plot(sim_time, acc_euler_deg(1,:), "LineWidth",1)
xlabel("Time [s]")
ylabel("Roll [d]")
legend("EKF", "EKF + bias estimation", "Roll from accelerometer")
title('EKF vs EKF with bias');


subplot(3,1,2);
plot(sim_time, euler_deg(2,:))
hold on
plot(sim_time, euler_deg_bias(2,:))
hold on
plot(sim_time, acc_euler_deg(2,:), "LineWidth",1)
xlabel("Time [s]")
ylabel("Pitch [d]")
legend("EKF", "EKF + bias estimation", "Pitch from accelerometer")
title('EKF vs EKF with bias');

subplot(3,1,3);
plot(sim_time, euler_deg(3,:))
hold on
plot(sim_time, euler_deg_bias(3,:))
hold on
plot(sim_time, acc_euler_deg(3,:))
xlabel("Time [s]")
ylabel("Yaw [d]")
legend("EKF", "EKF + bias estimation", "Yaw from accelerometer")
title('EKF vs EKF with bias');

