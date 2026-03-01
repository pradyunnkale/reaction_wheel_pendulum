function y = measurement(x)
    g = 9.81;
    theta = x(1);
    dtheta = x(2);
    dphi = x(4);

    y = zeros(2, 1);
    y(1) = g * sin(theta);
    y(2) = dtheta + dphi;
end