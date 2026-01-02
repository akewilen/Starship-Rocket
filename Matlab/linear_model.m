clc;
% Linearized rocket model around hover with height states
% State x = [p; q; r; phi; theta; psi; vx; vy; vz; z]
% Input u = [Fx; Fy; Fz; Mz]
% Outputs (example): y = [phi; theta; r; z; zdot; Fz]

rocket_model = importGeometry("simple_3d_rocket.stl");

m = 3;
Ix = 0.515;
Iy = 0.515;
Iz = 0.160;
g = 9.82;
h = 0.4345;
L = 0.4345; % lenght from rudder center to COM
Ts = 0.005;

Imatrix = diag([Ix,Iy,Iz]);

% --- Attitude State-Space Matrices ---

% State Vector: x_att = [p; q; r; phi; theta; psi] (6x1)
% Input Vector: u_att = [Mx; My; Mz] (3x1)

% A_att Matrix (6x6)
% Derived from the top-left 6x6 block of the original A matrix.
% Rows 1-3: Angular rate dynamics (dot(p), dot(q), dot(r))
% Rows 4-6: Kinematics (dot(phi)=p, dot(theta)=q, dot(psi)=r)

A_att = [
    0, 0, 0, ((m*g*L)/Ix), 0, 0;  % dot(p)
    0, 0, 0, 0, ((m*g*L)/Iy), 0;  % dot(q)
    0, 0, 0, 0, 0,            0;  % dot(r)
    1, 0, 0, 0, 0,            0;  % dot(phi)
    0, 1, 0, 0, 0,            0;  % dot(theta)
    0, 0, 1, 0, 0,            0   % dot(psi)
];

% B_att Matrix (6x3)
% Relates control moments [Mx; My; Mz] to angular rates [p; q; r]
B_att = [
    1/Ix, 0, 0;      % dot(p) from Mx
    0, 1/Iy, 0;      % dot(q) from My
    0, 0, 1/Iz;      % dot(r) from Mz
    0, 0, 0;         % dot(phi) (not directly affected by moment)
    0, 0, 0;         % dot(theta) (not directly affected by moment)
    0, 0, 0          % dot(psi) (not directly affected by moment)
];

% C_att Matrix (6x6)
% Assumes full state measurement (output is the state itself)
C_att = eye(6);

% D_att Matrix (6x3)
% Direct feedthrough matrix is zero
D_att = zeros(6, 3);

% Create the state-space object (Optional but recommended for analysis)
sys_att = ss(A_att, B_att, C_att, D_att);
sysd = c2d(sys_att, Ts);

% --- AUGMENTATION FOR INTEGRAL ACTION on Yaw Rate (r) ---

% C_I defines the error state for integration: dot(x_I) = 0*p + 0*q + (-1)*r + ...
C_I = [0, 0, 1, 0, 0, 0; % Yaw rate 'r' is the 3rd state
       0, 0, 0, 1, 0, 0;
       0, 0, 0, 0, 1, 0];

% A_aug (7x7)
A_aug = [A_att, zeros(6, 3);  % Top 6 rows
        C_I,   zeros(3,3)];             % Bottom row (Integral dynamics)

% B_aug (7x3)
B_aug = [B_att;               % Top 6 rows
        zeros(3, 3)];          % Bottom row (Input does not affect integral error derivative)


% Q_att: [p, q, r, phi, theta, psi]
Q_att_diag = diag([1000, 1000, 5000, 500003, 500003, 1000]); 

% Q_aug (7x7) - Add weight for the integral state (x_I)
% High weight on Q_I forces the integral error to zero quickly.
Q_I_weight = diag([1000 1000 1000]); % Tune this to control integral action aggressiveness
Q_aug = blkdiag(Q_att_diag, Q_I_weight);

% R_att (3x3) - Input cost [Mx, My, Mz]
R_att = 10000*diag([1, 1, 1]);

% --- COMPUTE LQI GAIN ---
%K_aug = lqr(A_aug, B_aug, Q_aug, R_att);
K_aug = lqrd(A_aug, B_aug, Q_aug, R_att,Ts);

% Separate the gains for implementation
K_att = K_aug(:, 1:6)  % Standard state feedback gain
K_I   = K_aug(:, 7)    % Integral gain


%-------------- Altitude control -----------------

% --- System Parameters (Placeholders - Replace with actual values) ---
% You need the mass (m) for the altitude dynamics.
% Mass of the vehicle in kg (Example value)
% Gravity constant

% --- Altitude State-Space Matrices ---

% State Vector: x_alt = [vz; z] (2x1)
% Input Vector: u_alt = [Fz_linearized] (1x1)

% A_alt Matrix (2x2)
A_alt = [
    0, 0;  % dot(vz) = 0*vz + 0*z
    1, 0   % dot(z)  = 1*vz + 0*z
];

% B_alt Matrix (2x1)
B_alt = [
    -1/m;  % dot(vz) affected by Fz_linearized/m
    0     % dot(z) not directly affected by Fz_linearized
];

% C_alt Matrix (2x2) - Full state measurement
C_alt = eye(2);

% D_alt Matrix (2x1) - Zero feedforward
D_alt = zeros(2, 1);

% Create the state-space object
sys_alt = ss(A_alt, B_alt, C_alt, D_alt, Ts);


% --- LQR Design Considerations for Altitude ---
% 1. Q_alt: Penalize state errors [vz, z]. Typically, you want low vz and low z error.
Q_alt = diag([
    10,  ... % Weight on vz (velocity damping)
    10 ... % Weight on z (position tracking - usually very high)
]);

% 2. R_alt: Penalize control effort [Fz_linearized]. 
R_alt = 1; % Weight on input Fz_linearized

% Compute LQR gain for altitude
K_alt = lqrd(A_alt, B_alt, Q_alt, R_alt, Ts);

disp('LQR Gain K_alt (1x2):');
disp(K_alt);

% --- Altitude Control Implementation ---
% Fz_linearized = -K_alt * (x_alt - x_alt_cmd)
% Fz_cmd = m*g + Fz_linearized
