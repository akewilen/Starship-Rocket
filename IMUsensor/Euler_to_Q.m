function q = Euler_to_Q(roll, pitch, yaw)
%Converts Euler angles to quaternion representation
 cr = cos(roll * 0.5);
 sr = sin(roll * 0.5);
 cp = cos(pitch * 0.5);
 sp = sin(pitch * 0.5);
 cy = cos(yaw * 0.5);
 sy = sin(yaw * 0.5);

 q.w = cr * cp * cy + sr * sp * sy;
 q.x = sr .* cp .* cy - cr .* sp .* sy;
 q.y = cr .* sp .* cy + sr .* cp .* sy;
 q.z = cr .* cp .* sy - sr .* sp .* cy;


