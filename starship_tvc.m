%% TVC Control Design - Workspace Setup
% Run this script to calculate LQR gains and load variables for Simulink.

clear; clc;

%% Model initialization
format long

% Defining our symbolics
syms Ctp Cty... % Control torque on pitch and yaw angles due to thrust
Rtp Rty Tp Ty real positive % Restoring torque due to gravity

% Defining the value of some of the variables (Place holder values)
m = 4; % Starship rocket mass in kg
g = -9.81; % Gravity in m/s^2
L_cg = 0.3; % Length from center of gravity to pivot point-base in m
I_pitch = 5; % Moment of inertia about the pitch axis 
I_yaw = 5.5; % Moment of inertia about the yaw axis

% Calculating the control and restoring torques that act on the rocket
Rtp = (m*g*L_cg)/I_pitch; % Restoring torque due to gravity for pitch
Rty = (m*g*L_cg)/I_yaw; % Restoring torque due to gravity for yaw 

% Define system matrices A, B, C, D for the continous-time state-space representation
A_sys = [0 1 0 0; 
         Rtp 0 0 0;
         0 0 0 1;
         0 0 Rty 0];

C_sys = eye(4);

D_sys = zeros(4, 2);
 
% 2. LQR WEIGHTS
% ==============
% Q: State Penalty [theta, theta_dot, psi, psi_dot]
Q = diag([1000, 10, 1000, 10]); 
% R: Control Penalty [alpha, beta]
R = diag([1, 1]);

% 4. GAIN SCHEDULING CALCULATION
% ==============================
T_min = 20;    
T_max = 100;   
T_schedule = linspace(T_min, T_max, 5); % The "x-axis" of our lookup table

% Storage for the K matrix (flattened 1x8 vectors)
K_schedule_data = zeros(length(T_schedule), 8);

% The axis for the 8 elements of the K matrix
Gain_Index = 1:8;

fprintf('Calculating Gains...\n');
for i = 1:length(T_schedule)
    T_curr = T_schedule(i);
    
    % Control Torque due to thrust
    Ctp_curr = (T_curr * L_cg) / I_pitch;
    Cty_curr = (T_curr * L_cg) / I_yaw;
    
    % B Matrix changes with Thrust
    B_curr = [ 0, 0;
               Ctp_curr, 0;
               0, 0;
               0, Cty_curr];
           
    % Calculate LQR Gain
    K_curr = lqr(A_sys, B_curr, Q, R)
    
    % Flatten 2x4 matrix to 1x8 vector for lookup table
    K_schedule_data(i, :) = K_curr(:)'
end

% Define B_sys for the Plant block (using T_max for simulation test)
Ctp_max = (T_max * L_cg) / I_pitch;
Cty_max = (T_max * L_cg) / I_yaw;
B_sys = [0, 0; Ctp_max, 0; 0, 0; 0, Cty_max]

fprintf('Done. Workspace is ready for Simulink.\n');

% reshape(K_schedule_data(5, :), 2, 4)