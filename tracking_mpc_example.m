%% Tracking MPC Example
% This example demonstrates MPC for trajectory tracking
% The system tracks a sinusoidal reference trajectory
%
% Author: Guang-Ze Yang
% Date: 2025

clear all;
close all;
clc;

%% System Definition: Inverted Pendulum on Cart (Linearized)
% State: [cart_position; cart_velocity; pendulum_angle; pendulum_angular_velocity]
% Input: Force on cart

% Physical parameters
M = 1.0;    % Cart mass (kg)
m = 0.1;    % Pendulum mass (kg)
l = 1.0;    % Pendulum length (m)
g = 9.81;   % Gravity (m/s^2)
b = 0.1;    % Damping coefficient

% Linearization around upright position (theta = 0)
p = M + m;
Ac = [0, 1, 0, 0;
      0, -b/p, -m*g/p, 0;
      0, 0, 0, 1;
      0, b/(l*p), g*(M+m)/(l*p), 0];

Bc = [0; 1/p; 0; -1/(l*p)];

% Output: cart position
Cc = [1, 0, 0, 0];
Dc = 0;

% Discretization
Ts = 0.05;
sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, Ts);

A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;

% System dimensions
nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);

%% MPC Parameters
N = 20;              % Prediction horizon
Q = diag([100, 1, 50, 1]);  % State weighting
R = 0.1;             % Input weighting
P = dare(A, B, Q, R);% Terminal cost

% Initial state
x0 = [0; 0; 0.1; 0]; % Small initial angle

% Simulation parameters
Tsim = 400;

% Constraints
u_min = -10.0;
u_max = 10.0;

%% Generate Reference Trajectory
t = (0:Tsim-1)*Ts;
% Sinusoidal reference for cart position
amplitude = 0.5;
frequency = 0.5;
r_traj = amplitude * sin(2*pi*frequency*t);

% Reference state trajectory (cart at r_traj, upright pendulum)
x_ref_traj = [r_traj; zeros(1, Tsim); zeros(1, Tsim); zeros(1, Tsim)];

%% MPC Tracking Simulation
x = x0;
x_history = zeros(nx, Tsim);
u_history = zeros(nu, Tsim);
y_history = zeros(ny, Tsim);

fprintf('Running Tracking MPC Simulation...\n');

for k = 1:Tsim
    % Get reference for prediction horizon
    ref_horizon = min(k+N-1, Tsim);
    x_ref = x_ref_traj(:, k:ref_horizon);
    
    % Pad if necessary
    if size(x_ref, 2) < N
        x_ref = [x_ref, repmat(x_ref(:, end), 1, N - size(x_ref, 2))];
    end
    
    % Solve tracking MPC problem
    [u_opt, exitflag] = solve_tracking_mpc(A, B, x, x_ref, N, Q, R, P, ...
                                           u_min, u_max);
    
    if exitflag < 0
        warning('MPC solver failed at step %d', k);
        u = 0;
    else
        u = u_opt(1);
    end
    
    % System update
    x = A*x + B*u;
    y = C*x;
    
    % Store history
    x_history(:, k) = x;
    u_history(:, k) = u;
    y_history(:, k) = y;
end

fprintf('Simulation complete!\n');

%% Visualization
figure('Position', [100, 100, 1200, 900]);

% Plot cart position
subplot(4,1,1);
plot(t, y_history, 'b-', 'LineWidth', 2);
hold on;
plot(t, r_traj, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Cart Position [m]');
legend('Actual', 'Reference', 'Location', 'best');
title('Tracking MPC - Cart Position');

% Plot tracking error
subplot(4,1,2);
tracking_error = y_history - r_traj;
plot(t, tracking_error, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Tracking Error [m]');
title('Tracking Error');

% Plot pendulum angle
subplot(4,1,3);
plot(t, x_history(3,:)*180/pi, 'g-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Pendulum Angle [deg]');
title('Pendulum Angle (should stay near zero)');

% Plot control input
subplot(4,1,4);
stairs(t, u_history, 'r-', 'LineWidth', 2);
hold on;
plot(t, u_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(t, u_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time [s]');
ylabel('Control Force [N]');
legend('Control Input', 'Constraints', 'Location', 'best');
title('Control Input');

% Compute performance metrics
fprintf('\n--- Performance Metrics ---\n');
fprintf('RMS Tracking Error: %.4f m\n', rms(tracking_error));
fprintf('Max Tracking Error: %.4f m\n', max(abs(tracking_error)));
fprintf('RMS Control Effort: %.4f N\n', rms(u_history));
fprintf('Max Pendulum Angle: %.2f deg\n', max(abs(x_history(3,:)))*180/pi);

%% Tracking MPC Solver Function
function [u_opt, exitflag] = solve_tracking_mpc(A, B, x0, X_ref, N, Q, R, P, ...
                                                 u_min, u_max)
    % Solve tracking MPC optimization problem with time-varying reference
    %
    % Minimize: sum_{i=0}^{N-1} [(x_i - x_ref_i)'*Q*(x_i - x_ref_i) + u_i'*R*u_i] 
    %           + (x_N - x_ref_N)'*P*(x_N - x_ref_N)
    % Subject to: x_{i+1} = A*x_i + B*u_i
    %             u_min <= u_i <= u_max
    
    nx = size(A, 1);
    nu = size(B, 2);
    
    % Build lifted system matrices
    Sx = zeros(nx*N, nx);
    Su = zeros(nx*N, nu*N);
    
    for i = 1:N
        Sx((i-1)*nx+1:i*nx, :) = A^i;
        for j = 1:i
            Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j)*B;
        end
    end
    
    % Build cost function matrices
    Q_bar = kron(eye(N-1), Q);
    Q_bar = blkdiag(Q_bar, P);
    R_bar = kron(eye(N), R);
    
    % Time-varying reference
    X_ref_vec = reshape(X_ref, nx*N, 1);
    
    % Quadratic cost
    H = Su'*Q_bar*Su + R_bar;
    f = Su'*Q_bar*(Sx*x0 - X_ref_vec);
    H = (H + H')/2;
    
    % Input constraints
    lb = u_min * ones(N*nu, 1);
    ub = u_max * ones(N*nu, 1);
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    [U_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    
    if exitflag < 0
        u_opt = zeros(N*nu, 1);
    else
        u_opt = reshape(U_opt, nu, N);
    end
end
