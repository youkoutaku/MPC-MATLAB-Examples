%% Simple MPC Example using Utility Functions
% This example demonstrates how to use the utility functions in mpc_utils.m
% to implement a basic MPC controller
%
% Author: Guang-Ze Yang
% Date: 2025

clear all;
close all;
clc;

%% System Definition
% Simple first-order system: x(k+1) = 0.9*x(k) + 0.1*u(k)
A = 0.9;
B = 0.1;
C = 1;

% System dimensions
nx = 1;
nu = 1;

%% Check System Properties
fprintf('=== System Analysis ===\n');
fprintf('System eigenvalues: %.3f\n', eig(A));
if abs(eig(A)) < 1
    fprintf('System is stable\n');
else
    fprintf('System is unstable\n');
end
fprintf('System is controllable: %d\n', rank(ctrb(A,B)) == nx);
fprintf('\n');

%% MPC Parameters
N = 10;         % Prediction horizon
Q = 10;         % State weight
R = 1;          % Input weight

% Compute terminal cost using utility function
P = compute_terminal_cost(A, B, Q, R);
fprintf('=== MPC Configuration ===\n');
fprintf('Prediction horizon: %d\n', N);
fprintf('Terminal cost P: %.3f\n', P);

% Check stability conditions
is_stable = check_mpc_stability(A, B, Q, R);
if is_stable
    fprintf('MPC stability conditions satisfied\n');
end
fprintf('\n');

%% Build MPC Matrices
% Build lifted matrices using utility function
[Sx, Su] = build_lifted_matrices(A, B, N);

% Build block diagonal weight matrices
Q_bar = kron(eye(N-1), Q);
Q_bar = blkdiag(Q_bar, P);
R_bar = kron(eye(N), R);

%% Simulation Setup
x0 = 5;         % Initial state
r = 1;          % Reference
Tsim = 30;      % Simulation steps

% Constraints
u_min = -2;
u_max = 2;

%% Run MPC Simulation
x = x0;
x_history = zeros(nx, Tsim);
u_history = zeros(nu, Tsim);

fprintf('=== Running Simulation ===\n');

for k = 1:Tsim
    % Reference trajectory
    x_ref = r * ones(N, 1);
    
    % Build cost function using utility function
    [H, f] = build_mpc_cost(Sx, Su, x, x_ref, Q_bar, R_bar);
    
    % Constraints
    lb = u_min * ones(N*nu, 1);
    ub = u_max * ones(N*nu, 1);
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    U_opt = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    
    % Apply first control input
    u = U_opt(1);
    
    % Update system
    x = A*x + B*u;
    
    % Store history
    x_history(k) = x;
    u_history(k) = u;
end

fprintf('Simulation complete!\n\n');

%% Compute Performance Metrics
x_ref_history = r * ones(1, Tsim);
metrics = compute_mpc_metrics(x_history, u_history, x_ref_history);

fprintf('=== Performance Metrics ===\n');
fprintf('State RMSE: %.4f\n', metrics.state_rmse);
fprintf('State Max Error: %.4f\n', metrics.state_max_error);
fprintf('Control RMS: %.4f\n', metrics.control_rms);
fprintf('Control Max: %.4f\n', metrics.control_max);
fprintf('Control Variation: %.4f\n', metrics.control_variation);
fprintf('Settling Time: %d steps\n', metrics.settling_time);
fprintf('\n');

%% Plot Results using Utility Function
t = 0:Tsim-1;
plot_mpc_results(t, x_history, u_history, r, u_min, u_max);
sgtitle('Simple MPC Example - First Order System');

%% Additional Analysis
figure('Position', [150, 150, 800, 400]);

% Plot tracking error
subplot(1,2,1);
tracking_error = x_history - r;
plot(t, tracking_error, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time Step');
ylabel('Tracking Error');
title('State Tracking Error');

% Plot control action vs constraints
subplot(1,2,2);
stairs(t, u_history, 'r-', 'LineWidth', 2);
hold on;
plot(t, u_max*ones(1,Tsim), 'k--', 'LineWidth', 1.5);
plot(t, u_min*ones(1,Tsim), 'k--', 'LineWidth', 1.5);
% Highlight constraint violations (if any)
violations = (u_history > u_max) | (u_history < u_min);
if any(violations)
    plot(t(violations), u_history(violations), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    legend('Control', 'Upper Limit', 'Lower Limit', 'Violations', 'Location', 'best');
else
    legend('Control', 'Upper Limit', 'Lower Limit', 'Location', 'best');
end
grid on;
xlabel('Time Step');
ylabel('Control Input');
title('Control Input with Constraints');

fprintf('All plots generated successfully!\n');
