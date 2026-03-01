function dxdt = dynamics(x, u, mr, mw, l, Iw)
    g = 9.81;
    theta = x(1);
    dtheta = x(2);
    dphi = x(4);

    dxdt = zeros(4, 1);
    dxdt(1) = dtheta;
    dxdt(2) = (((1/2)*mr*g*l + mw*g*l)*sin(theta) - u)/((1/3)*mr*l^2 + mw*l^2);
    dxdt(3) = dphi;
    dxdt(4) = u/Iw - dxdt(2);
end