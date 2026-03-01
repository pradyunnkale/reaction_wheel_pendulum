function [x_new, P_new] = ekf(f, xk, Pk, uk, yk, wk, h, Q, R, mr, mw, l, Iw, dt)
    g = 9.81;
    theta = xk(1);
    dtheta = xk(2);
    phi = xk(3);
    dphi = xk(4);

    F = [0, 1, 0, 0; ...
        ((1/2)*mr*g*l + mw*g*l)*cos(theta)/((1/3)*mr*l^2+mw*l^2), 0, 0, 0; ...
        0, 0, 0, 1; ...
        -((1/2)*mr*g*l + mw*g*l)*cos(theta)/((1/3)*mr*l^2+mw*l^2), 0, 0, 0; ...
        ];

    H = [g*cos(theta), 0, 0, 0; ...
         0, 1, 0, 1];

    Fk = expm(F*dt);

    x_pred = rk4(@(x, u) dynamics(x, u, mr, mw, l, Iw), xk, uk, wk, dt);
    P_pred = Fk * Pk * Fk' + Q;
    y_pred = h(x_pred);

    S = H * P_pred * H' + R;
    K = P_pred * H' / S;

    yinno = yk - y_pred;
    x_new = x_pred + K * yinno;
    P_new = (eye(length(P_pred)) - K * H) * P_pred;
end