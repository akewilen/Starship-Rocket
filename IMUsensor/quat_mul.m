function q_prod = quat_mul(q1, q2)
%This function mulitplies two quaternions 
r1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
r2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);

q_prod = [r1*r2 - x1*x2 - y1*y2 - z1*z2;
    r1*x2 + r2*x1 + y1*z2 - z1*y2;
    r1*y2 + r2*y1 + z1*x2 - x1*z2;
    r1*z2 + r2*z1 + x1*y2 - y1*x2];

if norm(q_prod)~= 0 %Normalize quaternion
    q_prod = q_prod/norm(q_prod);
end
