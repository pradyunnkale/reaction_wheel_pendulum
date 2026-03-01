function u = pid(x, dt)
    persistent integral prev_error
    if isempty(integral)
        integral = 0;
        prev_error = 0;
    end

    Kp = 50;
    Ki = 1;
    Kd = 50;
    u_max = 50;
    theta = x(1);
    dtheta = x(2);

    error = theta;
    integral = integral + error * dt;
    derivative = dtheta;

    u = (Kp * error + Ki * integral + Kd * derivative);
    u = max(-u_max, min(u_max, u));
    prev_error = error;
end