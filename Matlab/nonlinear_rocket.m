function dx = nonlinear_rocket(t, x, u, m, Ix, Iy, Iz, g, h)
% Nonlinear rocket model with height states z and zdot
% State x = [p; q; r; phi; theta; psi; vx; vy; vz; z; zdot]
% Input u = [Fx; Fy; Fz; Mz]

% Unpack states
p     = x(1);
q     = x(2);
r     = x(3);
phi   = x(4);
theta = x(5);
psi   = x(6);
vx    = x(7);
vy    = x(8);
vz    = x(9);
z     = x(10);
zdot  = x(11);

% Inputs
Fx = u(1);
Fy = u(2);
Fz = u(3);
Mz = u(4);

% Thrust offset moments (body frame)
Mx = -h * Fy;
My =  h * Fx;

% Rigid-body rotational Euler equations (full nonlinear)
pdot = (Mx + (Iy - Iz) * q * r) / Ix;
qdot = (My + (Iz - Ix) * p * r) / Iy;
rdot = (Mz + (Ix - Iy) * p * q) / Iz;

% Nonlinear Euler angle kinematics (ZYX / yaw-pitch-roll convention)
% phi_dot, theta_dot, psi_dot in terms of body rates p,q,r
phi_dot   = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
theta_dot = q * cos(phi) - r * sin(phi);
psi_dot   = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta);

% Rotation from body to inertial (ZYX: R = Rz(psi)*Ry(theta)*Rx(phi))
R = angle2dcm(psi, theta, phi, 'ZYX');  % MATLAB Aerospace toolbox function

% Body forces -> inertial
F_body = [Fx; Fy; Fz];
F_inertial = R * F_body;

% Translational accelerations (inertial frame)
vx_dot = F_inertial(1) / m;
vy_dot = F_inertial(2) / m;
vz_dot = F_inertial(3) / m + g;   % include gravity term in nonlinear model

% Height dynamics
z_dot = zdot;
zdot_dot = vz_dot;   % vertical accel equals derivative of zdot

% Pack derivative
dx = [pdot;
      qdot;
      rdot;
      phi_dot;
      theta_dot;
      psi_dot;
      vx_dot;
      vy_dot;
      vz_dot;
      z_dot;
      zdot_dot];
end
