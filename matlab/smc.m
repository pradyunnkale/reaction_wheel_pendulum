function u = smc(x)

    % Controller Initialization (SMC)
    lambda = 5;
    K = 20;
    phi_sat = 2.0;
    u_max = 50;

    theta = x(1);
    dtheta = x(2);
    
    % Angle Wrapping
    theta = mod(theta+pi, 2*pi) - pi;
    
    s = dtheta + lambda * theta;

    % Saturation Layer
    if abs(s) < phi_sat
        sat_val = s / phi_sat;
    else
        sat_val = sign(s);
    end

    u = -K * sat_val;
    u = max(-u_max, min(u_max, u));
end