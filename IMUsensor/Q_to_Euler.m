function [roll, pitch, yaw] = Q_to_Euler(q)
% Rotations in 3-2-1 sequence

 qw = q(1); qx = q(2); qy = q(3); qz = q(4);
 roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));

 term1 = sqrt(1 + 2*(qw*qy - qx*qz));
 term2 = sqrt(1 - 2*(qw*qy - qx*qz));

 pitch = -pi/2 + 2*atan2(term1, term2);

 yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));