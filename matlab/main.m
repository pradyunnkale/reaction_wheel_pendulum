% Initialization 
mr = 1; % Mass of the rod
mw = 1; % Mass of the wheel
l = 1; % Length of the rod
g = 9.81; % Acceleration due to gravity
r = 0.1;
Iw = (1/2) * mw * r^2; % Inertia of the wheel
x0 = [0.1; 0; 0; 0]; % Initial State
x0_est = [0.05; 0; 0; 0]; % Initial Guess
dt = 0.01; % Timestep

% Simulation
t = 0:dt:60;
N = numel(t);
x = zeros(4, N);
x_est = zeros(4, N);
y = zeros(2, N);
u = zeros(1, N);
x(:, 1) = x0;
x_est(:, 1) = x0_est;

P0 = diag([1, 1, 1, 1]);
Q = diag([1e-6, 1e-6, 1e-6, 1e-6]);
R = diag([1/4, 1/4]);

P = zeros(4, 4, N);
P(:,:,1) = P0;

for i = 2:N
    L = chol(Q, "lower");
    wk = L * randn(4,1);
    vk = sqrtm(R) * randn(2,1);
    %uk = smc(x_est(:,i-1));
    %u(:,i-1) = uk;
    uk = pidcontrol(x_est(:,i-1), dt);
    u(:,i-1) = uk;

    x(:,i) = rk4(@(x, u) dynamics(x, u, mr, mw, l, Iw), x(:,i-1), uk, wk, dt);
    y(:,i) = measurement(x(:,i)) + vk;

    [x_est(:,i), P(:,:,i)] = ekf(@(x, u) dynamics(x, u, mr, mw, l, Iw), x_est(:,i-1), P(:,:,i-1), uk, y(:,i), wk, ...
                             @measurement, Q, R, mr, mw, l, Iw, dt);
end

figure;
hold on;

plot(t, x(1,:), 'b', 'DisplayName', 'Theta');
plot(t, x(2,:), 'r', 'DisplayName', 'Vel Theta');
plot(t, x_est(1,:), 'g--', 'DisplayName', 'Theta Est');
plot(t, x_est(2,:), 'm--', 'DisplayName', 'Vel Theta Est');

plot(t, x(3,:), 'c', 'DisplayName', 'Phi');
plot(t, x(4,:), 'y', 'DisplayName', 'Vel Phi');
plot(t, x_est(3,:), 'k--', 'DisplayName', 'Phi Est');
plot(t, x_est(4,:), '--', 'DisplayName', 'Vel Phi Est');

hold off;
title("State vs Time [s]");
xlabel("Time [s]");
ylabel("State");
legend;
grid on;

x_err = x - x_est;

figure;
plot(t, x_err);
title("State vs Time [s]");
xlabel("Time [s]");
ylabel("State");
legend("Theta err", "Vel Theta err", "Phi err", "Vel Phi err");
grid on;

pendanimation(x, l, r, N);