function xk1 = rk4(func, xk, uk, wk, dt)
    k1 = func(xk, uk);
    k2 = func(xk + dt/2 * k1, uk);
    k3 = func(xk + dt/2 * k2, uk);
    k4 = func(xk + dt * k3, uk);
    xk1 = xk + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4) + wk;
end