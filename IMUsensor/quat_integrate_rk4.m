function q_next = quat_integrate_rk4(q, w, dt)
% RK4 quaternion integration using Somega(w)

    f = @(qq, ww) 0.5 * Somega(ww) * qq;

    k1 = f(q,          w);
    k2 = f(q + 0.5*dt*k1, w);
    k3 = f(q + 0.5*dt*k2, w);
    k4 = f(q + dt*k3,     w);

    q_next = q + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    q_next = q_next / norm(q_next);
end
