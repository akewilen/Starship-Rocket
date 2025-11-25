function q_next = quat_propagate(q, omega, dt)
%QUAT_PROPAGATE Propagate quaternion with body angular velocity.
%
%   q_next = quat_propagate(q, omega, dt)
%
%   q     : current quaternion [qw; qx; qy; qz], unit norm
%   omega : body angular velocity [wx; wy; wz] in rad/s
%   dt    : timestep in seconds
%
%   q_next: propagated, normalized quaternion

    % Build Omega(omega) matrix for q_dot = 0.5 * Omega * q
    wx = omega(1);
    wy = omega(2);
    wz = omega(3);

    Omega = [  0,  -wx,  -wy,  -wz;
              wx,    0,   wz,  -wy;
              wy,  -wz,    0,   wx;
              wz,   wy,  -wx,    0 ];

    % Quaternion derivative
    q_dot = 0.5 * Omega * q;

    % Forward Euler integration
    q_next = q + q_dot * dt;

    % Normalize to avoid drift
    q_next = q_next / norm(q_next);
end