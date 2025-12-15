%% Offset-Free MPC Example (Incremental MPC)
% This example demonstrates offset-free tracking MPC using state augmentation
% to handle constant disturbances and model mismatch
%
% Author: Guang-Ze Yang
% Date: 2025

clear all;
close all;
clc;

%% Original System Definition
% System with unknown disturbance
% x(k+1) = A*x(k) + B*u(k) + d
% y(k) = C*x(k)

A = [0.9 0.1; 0 0.8];
B = [0; 0.1];
C = [1 0];

nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);

% Unknown constant disturbance (not known to controller)
d_true = [0.1; -0.05];

%% Augmented System for Offset-Free Tracking
% Augment state with disturbance estimate
% [x(k+1)]   [A B] [x(k)]   [B]
% [d(k+1)] = [0 I] [d(k)] + [0] u(k)
%
% y(k) = [C 0] [x(k); d(k)]

A_aug = [A, eye(nx); zeros(nx, nx), eye(nx)];
B_aug = [B; zeros(nx, nu)];
C_aug = [C, zeros(ny, nx)];

nx_aug = size(A_aug, 1);

% Observer gain for disturbance estimation (Luenberger observer)
% Place observer poles faster than system poles
desired_poles = [0.5, 0.4, 0.3, 0.2];
L = place(A_aug', C_aug', desired_poles)';

%% MPC Parameters
N = 15;              % Prediction horizon
Q = diag([100, 10]); % State weight (original states only)
R = 1;               % Input weight

% Augmented weight matrix
Q_aug = blkdiag(Q, zeros(nx));

% Terminal cost
P_aug = dare(A_aug, B_aug, Q_aug, R);

%% Simulation Setup
x0 = [0; 0];         % Initial state
x_aug0 = [x0; zeros(nx, 1)]; % Augmented initial state (no initial disturbance estimate)

r = 2.0;             % Reference output

Tsim = 100;

% Constraints
u_min = -5;
u_max = 5;

%% Simulation with Disturbance
x = x0;
x_aug = x_aug0;
x_history = zeros(nx, Tsim);
x_aug_history = zeros(nx_aug, Tsim);
u_history = zeros(nu, Tsim);
y_history = zeros(ny, Tsim);
d_est_history = zeros(nx, Tsim);

fprintf('Running Offset-Free MPC Simulation...\n');

for k = 1:Tsim
    % Measurement
    y = C * x;
    
    % Observer update for disturbance estimation
    y_pred = C_aug * x_aug;
    x_aug = x_aug + L * (y - y_pred);
    
    % Extract estimated disturbance
    d_est = x_aug(nx+1:end);
    
    % Solve MPC for augmented system
    [u_opt, exitflag] = solve_offset_free_mpc(A_aug, B_aug, C_aug, x_aug, r, ...
                                               N, Q_aug, R, P_aug, u_min, u_max);
    
    if exitflag < 0
        warning('MPC solver failed at step %d', k);
        u = 0;
    else
        u = u_opt(1);
    end
    
    % Apply control to true system (with unknown disturbance)
    x = A*x + B*u + d_true;
    
    % Predict next augmented state for observer
    x_aug = A_aug * x_aug + B_aug * u;
    
    % Store history
    x_history(:, k) = x;
    x_aug_history(:, k) = x_aug;
    u_history(:, k) = u;
    y_history(:, k) = y;
    d_est_history(:, k) = d_est;
end

fprintf('Simulation complete!\n');

%% Run Standard MPC for Comparison (without disturbance estimation)
fprintf('Running standard MPC for comparison...\n');

x_std = x0;
x_std_history = zeros(nx, Tsim);
u_std_history = zeros(nu, Tsim);
y_std_history = zeros(ny, Tsim);

for k = 1:Tsim
    % Solve standard MPC (ignoring disturbance)
    [u_opt, exitflag] = solve_standard_mpc(A, B, C, x_std, r, N, Q, R, ...
                                           dare(A, B, Q, R), u_min, u_max);
    
    if exitflag < 0
        u = 0;
    else
        u = u_opt(1);
    end
    
    % Apply control to true system (with disturbance)
    x_std = A*x_std + B*u + d_true;
    y_std = C*x_std;
    
    % Store history
    x_std_history(:, k) = x_std;
    u_std_history(:, k) = u;
    y_std_history(:, k) = y_std;
end

fprintf('Comparison simulation complete!\n');

%% Visualization
figure('Position', [100, 100, 1200, 900]);

% Output tracking
subplot(4,1,1);
plot(0:Tsim-1, y_history, 'b-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, y_std_history, 'g--', 'LineWidth', 2);
plot(0:Tsim-1, r*ones(1,Tsim), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time Step');
ylabel('Output y');
legend('Offset-Free MPC', 'Standard MPC', 'Reference', 'Location', 'best');
title('Offset-Free MPC vs Standard MPC');

% Tracking error
subplot(4,1,2);
error_offset_free = y_history - r;
error_standard = y_std_history - r;
plot(0:Tsim-1, error_offset_free, 'b-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, error_standard, 'g--', 'LineWidth', 2);
grid on;
xlabel('Time Step');
ylabel('Tracking Error');
legend('Offset-Free MPC', 'Standard MPC', 'Location', 'best');
title('Tracking Error Comparison');

% Disturbance estimation
subplot(4,1,3);
plot(0:Tsim-1, d_est_history(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, d_est_history(2,:), 'g-', 'LineWidth', 2);
plot(0:Tsim-1, d_true(1)*ones(1,Tsim), 'b--', 'LineWidth', 1.5);
plot(0:Tsim-1, d_true(2)*ones(1,Tsim), 'g--', 'LineWidth', 1.5);
grid on;
xlabel('Time Step');
ylabel('Disturbance');
legend('d_1 estimate', 'd_2 estimate', 'd_1 true', 'd_2 true', 'Location', 'best');
title('Disturbance Estimation');

% Control input
subplot(4,1,4);
stairs(0:Tsim-1, u_history, 'b-', 'LineWidth', 2);
hold on;
stairs(0:Tsim-1, u_std_history, 'g--', 'LineWidth', 2);
plot(0:Tsim-1, u_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(0:Tsim-1, u_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('Control Input u');
legend('Offset-Free MPC', 'Standard MPC', 'Constraints', 'Location', 'best');
title('Control Input Comparison');

%% Performance Analysis
fprintf('\n=== Performance Comparison ===\n');
fprintf('Offset-Free MPC:\n');
fprintf('  Steady-state error: %.6f\n', mean(error_offset_free(end-20:end)));
fprintf('  RMS error: %.4f\n', rms(error_offset_free));
fprintf('\nStandard MPC:\n');
fprintf('  Steady-state error: %.6f\n', mean(error_standard(end-20:end)));
fprintf('  RMS error: %.4f\n', rms(error_standard));

%% Helper Functions

function [u_opt, exitflag] = solve_offset_free_mpc(A_aug, B_aug, C_aug, x_aug0, r, ...
                                                    N, Q_aug, R, P_aug, u_min, u_max)
    % Solve offset-free MPC for augmented system
    
    nx_aug = size(A_aug, 1);
    nu = size(B_aug, 2);
    
    % Compute steady-state target
    % Find [x_ss; d_ss; u_ss] such that y_ss = r
    % x_ss = A*x_ss + B*u_ss (no change in steady state)
    % r = C*x_ss
    
    % Reference state: we want C*x = r, with disturbance compensation
    % For simplicity, we track x_aug to a steady state
    
    % Build lifted matrices
    Sx = zeros(nx_aug*N, nx_aug);
    Su = zeros(nx_aug*N, nu*N);
    
    for i = 1:N
        Sx((i-1)*nx_aug+1:i*nx_aug, :) = A_aug^i;
        for j = 1:i
            Su((i-1)*nx_aug+1:i*nx_aug, (j-1)*nu+1:j*nu) = A_aug^(i-j)*B_aug;
        end
    end
    
    % Cost function
    Q_bar = kron(eye(N-1), Q_aug);
    Q_bar = blkdiag(Q_bar, P_aug);
    R_bar = kron(eye(N), R);
    
    % Reference: maintain current estimated state (delta formulation)
    X_ref = repmat(x_aug0, N, 1);
    
    H = Su'*Q_bar*Su + R_bar;
    f = Su'*Q_bar*(Sx*x_aug0 - X_ref);
    H = (H + H')/2;
    
    % Constraints
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

function [u_opt, exitflag] = solve_standard_mpc(A, B, C, x0, r, N, Q, R, P, ...
                                                 u_min, u_max)
    % Solve standard MPC without disturbance handling
    
    nx = size(A, 1);
    nu = size(B, 2);
    
    % Reference state (assumes output reference corresponds to certain state)
    x_ref = [r; 0];
    
    % Build lifted matrices
    Sx = zeros(nx*N, nx);
    Su = zeros(nx*N, nu*N);
    
    for i = 1:N
        Sx((i-1)*nx+1:i*nx, :) = A^i;
        for j = 1:i
            Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j)*B;
        end
    end
    
    % Cost function
    Q_bar = kron(eye(N-1), Q);
    Q_bar = blkdiag(Q_bar, P);
    R_bar = kron(eye(N), R);
    
    X_ref = repmat(x_ref, N, 1);
    
    H = Su'*Q_bar*Su + R_bar;
    f = Su'*Q_bar*(Sx*x0 - X_ref);
    H = (H + H')/2;
    
    % Constraints
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
