%% TVC Control Design - 6 State Workspace Setup (Includes Roll) and 4 inputs
% Updates: Adds Roll dynamics (phi, phi_dot) controlled by differential thrust.

clear; clc;

%% Model initialization
format long

% Defining our symbolics
syms Ctp Cty... % Control torque on pitch and yaw angles due to thrust
Rtp Rty Tp Ty real positive % Restoring torque due to gravity

% Defining the value of some of the model variables
m = 2.5;          % Mass (kg)
g = 9.81;      % Gravity in m/s^2
L_cg = 0.3;     % CG to Gimbal distance (m)
I_pitch = 0.3;    % Moment of inertia about the pitch axis (kg*m^2)
I_yaw = 0.35;    % Moment of inertia about the yaw axis (kg*m^2)
I_roll = 0.4;   % New: Roll Moment of Inertia (kg*m^2) - Assume much smaller
tau_motor = 50; % Time constant in seconds (e.g., 50ms to reach 63% of new thrust command)

% Simulation variables
launchtime = 2; % Seconds
Total_Simulation_Time = 21; % Seconds
wind_gust_duration = 2; % seconds
pitch_offset_demand = 5; % degrees
pitch_demand_conversion = pitch_offset_demand * pi/180; % Radians
roll_demand = 20; % degrees
roll_demand_conversion = roll_demand * pi/180; % Radians



h_max = 4; % Maximum altitude for rocket model flight
ascent = linspace(0, h_max, 5);
descent = linspace(h_max, 0, 5);
flightplan_height = [ascent, descent];
Time_Vector = linspace(launchtime, Total_Simulation_Time, length(flightplan_height));
% Flight_time = linspace(0, T_max, 14);

% COAXIAL PARAMETERS
% ------------------
% Prop_Torque_Factor: How much Torque (Nm) is generated per Newton of Thrust difference.
% This replaces the "Distance between props".
% Value depends on prop pitch/diameter. Est: 0.02 (2% of thrust becomes torque).
K_torque  = 0.02;  

% Calculating the control and restoring torques that act on the rocket
Rtp = -(m*g*L_cg)/I_pitch; % Restoring torque due to gravity for pitch
Rty = -(m*g*L_cg)/I_yaw; % Restoring torque due to gravity for yaw 


K_phi = K_torque / I_roll;      % Roll Control Authority term (Constant)

% Define system matrices A, B, C, D for the continous-time state-space representation
% Extended states = [phi, phi_dot, theta, theta_dot, psi, psi_dot];

A_sys = [ 0, 1, 0, 0, 0, 0;  % phi
          0, 0, 0, 0, 0, 0;  % d_phi
          0, 0, 0, 1, 0, 0;  % theta
          0, 0, Rtp, 0, 0, 0; % d_theta
          0, 0, 0, 0, 0, 1;  % psi
          0, 0, 0, 0, Rty, 0];% d_psi

C_sys = eye(6); % 6 states are outputs

D_sys = zeros(6, 4); % 4 inputs
 
% 2. LQR WEIGHTS
% ==============
% Q: State Penalty [phi, phi_dot, theta, theta_dot, psi, psi_dot]
Q = diag([100, 1, 1000, 10, 1000, 10]); 
% R: Control Penalty [T1, T2, alpha, beta]
R = diag([5, 5, 1, 1]); 

% 4. GAIN SCHEDULING CALCULATION
% ==============================  
%T_schedule = linspace(T_min, T_max, 10); % The "x-axis" of our lookup table
% F_min = 0; % The starting value for applied Thrust force - kgf
T_schedule = g * [0.11, 0.43, 0.95, 1.5, 1.91, 2.45, 3.1, 3.5, 3.9, 4.23]; %m/s^2 * kgf = N
T_max = T_schedule(end); % N - Newton

% Storage for the K matrix (flattened 1x8 vectors)
K_schedule_data = zeros(length(T_schedule), 24); % New size: 3x6 = 18 elements

% The axis for the 18 elements of the K matrix
Gain_Index = 1:24;

fprintf('Calculating the LQR Gains...\n');
for i = 1:length(T_schedule)
    T_curr = T_schedule(i);
    
    % Control Torque due to thrust
    Ctp_curr = (T_curr * L_cg) / I_pitch;
    Cty_curr = (T_curr * L_cg) / I_yaw;
    
    % B Matrix (6x4) changes with Thrust
    % Cols: [T1, T2, Alpha, Beta]
    B_curr = [ 0,        0,     0,        0;
               K_phi,   -K_phi, 0,        0; % Roll dynamics
               0,        0,     0,        0;
               0,        0,     Ctp_curr, 0; % Pitch dynamics
               0,        0,     0,        0;
               0,        0,     0,        Cty_curr]; % Yaw dynamics
           
    % Calculate LQR Gain
    fprintf('The LQR gain for T_%d = %.2f.\n', i, T_curr);
    K_curr = lqr(A_sys, B_curr, Q, R)
       
    % Flatten 3x6 matrix to 1x18 vector (row-wise)
    K_schedule_data(i, :) = K_curr(:)';
end

% 5. FINAL WORKSPACE VARS for Simulink
% Define B_sys for the Plant block (using T_max for simulation test)
Ctp_max = (T_max * L_cg) / I_pitch;
Cty_max = (T_max * L_cg) / I_yaw;
B_sys = [0, 0, 0, 0; K_phi, -K_phi, 0, 0; 0, 0, 0, 0; 0, 0, Ctp_max, 0; 0, 0, 0, 0; 0, 0, 0, Cty_max];

fprintf('Done. Workspace is ready for Simulink.\n');

% reshape(K_schedule_data(5, :), 2, 4)