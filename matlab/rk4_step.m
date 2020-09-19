function x_next = rk4_step(x, u_const, A, B, C, D, dt)
    k1 = A*x + B*u_const;
    k2 = A*(x + 0.5*dt*k1) + B*u_const;
    k3 = A*(x + 0.5*dt*k2) + B*u_const;
    k4 = A*(x + dt*k3);
    x_next = x + (1/6)*dt*(k1 + 2*k2 + 2*k3 + k4);
end
