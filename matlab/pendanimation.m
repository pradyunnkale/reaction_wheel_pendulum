function pendanimation(x, l, r, N)
figure;
hold on;
axis equal;
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
grid on;
title("RWIP Animation");

for i = 1:5:N
    cla;

    theta = x(1, i);
    phi = x(3, i);

    % Rod endpoints
    rod_x = [0, l*sin(theta)];
    rod_y = [0, l*cos(theta)];

    % Wheel center
    wx = l*sin(theta);
    wy = l*cos(theta);

    % Draw rod
    plot(rod_x, rod_y, 'w-', 'LineWidth', 3);

    % Draw pivot
    plot(0, 0, 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'w');

    % Draw wheel as circle
    theta_circle = linspace(0, 2*pi, 50);
    plot(wx + r*cos(theta_circle), wy + r*sin(theta_circle), 'b-', 'LineWidth', 2);

    % Draw spoke to visualize wheel rotation (phi)
    spoke_x = [wx, wx + r*cos(theta + phi)];
    spoke_y = [wy, wy + r*sin(theta + phi)];
    plot(spoke_x, spoke_y, 'r-', 'LineWidth', 2);

    drawnow;
end
end