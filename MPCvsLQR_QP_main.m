%% MPC vs LQR Performance Comparison
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   Comparative analysis of MPC and LQR control strategies
%   for trajectory tracking with identical weight matrices.
%
% Features:
%   - Fair comparison with same Q, R weights
%   - Input saturation handling (±5 m/s²)
%   - Quantitative performance metrics
%   - Side-by-side visualization
%
% Requirements:
%   - MATLAB R2020b or later
%   - Control System Toolbox (dlqr)
%   - Optimization Toolbox (quadprog)
%   - lib/ folder in path
%
% Usage:
%   addpath('lib')
%   run('MPCvsLQR_QP_main.m')
%----------------------------------------------------------%
clear;
close all;
clc;
addpath('lib');  % Add library folder to path
set(0, 'DefaultAxesFontName', 'Times New Roman')
set(0, 'DefaultAxesFontSize', 14)

%% System Model
A = [0 1 ; 0 0];
n= size(A,1);
B = [0; 1];
p = size(B,2);
C = [1 0; 0 1];  % Measure both position and velocity
D = 0;

% the simulation time
Time = 20;
% the sampling time
ts = 0.05;
% the number of the simulation steps
k_steps = Time/ts;

sys_d = c2d(ss(A, B, C, D), ts); %discretization of continuous time system
A = sys_d.a; %discretized system matrix A
B = sys_d.b; %discretized input matrix B

%% Weight matrix
Q_mpc = [1 0; 0 0.2]; % 100, 10, 1
Qf_mpc= 80*[1 0; 0 0.2]; % 100, 10, 1
R_mpc = 10; % 1, 100

Q_lqr = [1 0; 0 0.2]; % 100, 10, 1
Qf_lqr= 80*[1 0; 0 0.2]; % 100, 10, 1
R_lqr = 10; % 1, 100

%% Initialization
% the number of the prediction horizon
Np = 30;
% the state of the system
x0 = [0; 0];
% the state of the system by LQR
x_lqr = zeros(n,k_steps+1);
x_lqr(:, 1) = x0;
% the state of the system by MPC
x_mpc = zeros(n,k_steps+1);
x_mpc(:, 1) = x0;
% the measurement of the system
ny = size(C,1);
y_lqr = zeros(ny, k_steps+1);
y_mpc = zeros(ny, k_steps+1);
y_lqr(:, 1) = C*x_lqr(:, 1);
y_mpc(:, 1) = C*x_mpc(:, 1);

% the input by LQR
u_lqr = zeros(p,k_steps);
% the input by MPC
u_mpc = zeros(p,k_steps);

umax = 5;
lb = -umax * ones(Np*p,1);
ub =  umax * ones(Np*p,1);

d = zeros(n,k_steps); % disturbance

%% Generate time-varying reference trajectory
t_ref = 0:ts:Time;  % time vector

% Sinusoidal trajectory (same as MPC_trajectory_main.m)
xr_traj = zeros(n, k_steps+1);
xr_traj(1,:) = 5 * sin(0.5*t_ref) + 10;  % position: amplitude=5, freq=0.5, offset=10
xr_traj(2,:) = 5 * 0.5 * cos(0.5*t_ref);  % velocity: derivative of position

%% Riccati equation for LQR (regulator design)
[K_lqr, S, CLP] = dlqr(A,B,Q_lqr,R_lqr); % Linear-quadratic regulator design for discrete-time systems.

%% MPC Setup for Trajectory Tracking
% Build prediction matrices using QP_Transform
[Sx, Su, Qp, Rp, ~, H] = QP_Transform(A, B, Q_mpc, R_mpc, Qf_mpc, Np);

% Build Iu matrix (effect of u(k-1) on future states)
Iu = zeros(Np*n, p);
for i = 1:Np
    temp = zeros(n, p);
    for j = 0:i-1
        temp = temp + A^j * B;
    end
    Iu((i-1)*n+1:i*n, :) = temp;
end

% Ensure H is symmetric
H = (H + H')/2;

%%  Simulation - discrete time system
options = optimoptions('quadprog','Display','off');

for k = 1 : k_steps
    %% disturbance
    %d(:, k) = [0; 0]; % no disturbance
    %d(:, k) = [sin(k/ts+x_mpc(1,k)), 0.1*cos(k/ts-x_mpc(2,k))]; % disturbance
    
    %% MPC - Trajectory Tracking
    % Get previous control input
    if k == 1
        u_prev = 0;
    else
        u_prev = u_mpc(:,k-1);
    end
    
    % Build reference trajectory for prediction horizon
    Xr = zeros(Np*n, 1);
    for i = 1:Np
        if k+i <= k_steps+1
            Xr((i-1)*n+1:i*n) = xr_traj(:, k+i);
        else
            Xr((i-1)*n+1:i*n) = xr_traj(:, end);
        end
    end
    
    % Compute the linear term: f = Su'*Qp*(Sx*x(k) + Iu*u_prev - Xr)
    f = Su'*Qp*(Sx*x_mpc(:,k) + Iu*u_prev - Xr);
    
    % Solve QP: min 0.5*ΔU'*H*ΔU + f'*ΔU subject to constraints
    [DU, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    
    if exitflag ~= 1
        warning('MPC Quadprog did not converge at step %d', k);
    end
    
    % Extract first control increment and update control
    du = DU(1:p);
    u_mpc(:,k) = u_prev + du;
    
    % Apply saturation manually if needed (redundant with constraints, but safe)
    u_mpc(:,k) = max(min(u_mpc(:,k), umax), -umax);
    
    % System update
    x_mpc(:,k+1) = A * x_mpc(:,k) + B * u_mpc(:,k) + d(:,k);
    y_mpc(:,k) = C * x_mpc(:,k);
    
    %% LQR - Error Feedback for Tracking
    % LQR regulates error to zero: e(k) = x(k) - xr(k)
    e_lqr = x_lqr(:,k) - xr_traj(:,k);
    
    % Control law: u = -K*e (error feedback)
    u_lqr(:,k) = -K_lqr * e_lqr;
    
    % Apply saturation
    u_lqr(:,k) = max(min(u_lqr(:,k), umax), -umax);
    
    % System update
    x_lqr(:,k+1) = A * x_lqr(:,k) + B * u_lqr(:,k) + d(:,k);
    y_lqr(:,k) = C * x_lqr(:,k);
end

%% Performance Analysis
% Create time vectors
t_state = 0:ts:Time;
t_input = 0:ts:Time-ts;

% Calculate tracking errors
error_pos_lqr = x_lqr(1,:) - xr_traj(1,:);
error_vel_lqr = x_lqr(2,:) - xr_traj(2,:);
error_pos_mpc = x_mpc(1,:) - xr_traj(1,:);
error_vel_mpc = x_mpc(2,:) - xr_traj(2,:);

% Performance metrics
RMSE_pos_lqr = sqrt(mean(error_pos_lqr.^2));
RMSE_vel_lqr = sqrt(mean(error_vel_lqr.^2));
RMSE_pos_mpc = sqrt(mean(error_pos_mpc.^2));
RMSE_vel_mpc = sqrt(mean(error_vel_mpc.^2));

Max_error_pos_lqr = max(abs(error_pos_lqr));
Max_error_vel_lqr = max(abs(error_vel_lqr));
Max_error_pos_mpc = max(abs(error_pos_mpc));
Max_error_vel_mpc = max(abs(error_vel_mpc));

fprintf('\n=== Performance Comparison: MPC vs LQR ===\n');
fprintf('Position Tracking:\n');
fprintf('  LQR  - RMSE: %.4f m, Max Error: %.4f m\n', RMSE_pos_lqr, Max_error_pos_lqr);
fprintf('  MPC  - RMSE: %.4f m, Max Error: %.4f m\n', RMSE_pos_mpc, Max_error_pos_mpc);
fprintf('Velocity Tracking:\n');
fprintf('  LQR  - RMSE: %.4f m/s, Max Error: %.4f m/s\n', RMSE_vel_lqr, Max_error_vel_lqr);
fprintf('  MPC  - RMSE: %.4f m/s, Max Error: %.4f m/s\n', RMSE_vel_mpc, Max_error_vel_mpc);

%% Plot Results
% Define consistent color scheme
color_ref = 'r';                    % Reference: Red
color_mpc = [0.00 0.45 0.74];       % MPC: Blue
color_lqr = 'g';                    % LQR: Green
color_constraint = [0.5 0.5 0.5];   % Constraints: Gray
color_start = 'g';                  % Start marker: Green
color_end = 'r';                    % End marker: Red

figure('Position', [50, 50, 1400, 900], 'Color', 'w')

% Position tracking
subplot(3, 2, 1);
hold on;
plot(t_state, xr_traj(1,:), 'Color', color_ref, 'LineStyle', '--', 'LineWidth', 2);
plot(t_state, x_lqr(1,:), 'Color', color_lqr, 'LineWidth', 2);
plot(t_state, x_mpc(1,:), 'Color', color_mpc, 'LineWidth', 2.5);
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Position (m)', 'FontWeight', 'bold')
legend('Reference', 'LQR', 'MPC', 'Location', 'best')
title('Position Tracking Comparison', 'FontWeight', 'bold')
hold off;

% Velocity tracking
subplot(3, 2, 3);
hold on;
plot(t_state, xr_traj(2,:), 'Color', color_ref, 'LineStyle', '--', 'LineWidth', 2);
plot(t_state, x_lqr(2,:), 'Color', color_lqr, 'LineWidth', 2);
plot(t_state, x_mpc(2,:), 'Color', color_mpc, 'LineWidth', 2.5);
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Velocity (m/s)', 'FontWeight', 'bold')
legend('Reference', 'LQR', 'MPC', 'Location', 'best')
title('Velocity Tracking Comparison', 'FontWeight', 'bold')
hold off;

% Control input comparison
subplot(3, 2, 5);
hold on;
stairs(t_input, u_lqr(1,:), 'Color', color_lqr, 'LineWidth', 2);
stairs(t_input, u_mpc(1,:), 'Color', color_mpc, 'LineWidth', 2.5);
yline(umax, 'Color', color_constraint, 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'Constraint');
yline(-umax, 'Color', color_constraint, 'LineStyle', ':', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 1);
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Acceleration (m/s^2)', 'FontWeight', 'bold')
legend('LQR', 'MPC', 'Constraints', 'Location', 'best')
title('Control Input Comparison', 'FontWeight', 'bold')
xlim([0 Time])
hold off;

% Position error comparison
subplot(3, 2, 2);
hold on;
plot(t_state, error_pos_lqr, 'Color', color_lqr, 'LineWidth', 2, 'DisplayName', 'LQR');
plot(t_state, error_pos_mpc, 'Color', color_mpc, 'LineWidth', 2.5, 'DisplayName', 'MPC');
yline(0, 'k--', 'LineWidth', 1);
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Error (m)', 'FontWeight', 'bold')
legend('Location', 'best')
title('Position Tracking Error', 'FontWeight', 'bold')
text(0.02, 0.98, sprintf('LQR RMSE: %.3f m\nMPC RMSE: %.3f m', RMSE_pos_lqr, RMSE_pos_mpc), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'FontSize', 12, 'FontWeight', 'bold');
hold off;

% Velocity error comparison
subplot(3, 2, 4);
hold on;
plot(t_state, error_vel_lqr, 'Color', color_lqr, 'LineWidth', 2, 'DisplayName', 'LQR');
plot(t_state, error_vel_mpc, 'Color', color_mpc, 'LineWidth', 2.5, 'DisplayName', 'MPC');
yline(0, 'k--', 'LineWidth', 1);
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Error (m/s)', 'FontWeight', 'bold')
legend('Location', 'best')
title('Velocity Tracking Error', 'FontWeight', 'bold')
text(0.02, 0.98, sprintf('LQR RMSE: %.3f m/s\nMPC RMSE: %.3f m/s', RMSE_vel_lqr, RMSE_vel_mpc), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'FontSize', 12, 'FontWeight', 'bold');
hold off;

% Phase portrait comparison
subplot(3, 2, 6);
hold on;
plot(xr_traj(1,:), xr_traj(2,:), 'Color', color_ref, 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Reference');
plot(x_lqr(1,:), x_lqr(2,:), 'Color', color_lqr, 'LineWidth', 2, 'DisplayName', 'LQR');
plot(x_mpc(1,:), x_mpc(2,:), 'Color', color_mpc, 'LineWidth', 2.5, 'DisplayName', 'MPC');
plot(x0(1), x0(2), 'o', 'Color', color_start, 'MarkerSize', 10, 'MarkerFaceColor', color_start, 'DisplayName', 'Start');
plot(xr_traj(1,end), xr_traj(2,end), 's', 'Color', color_end, 'MarkerSize', 10, 'MarkerFaceColor', color_end, 'DisplayName', 'End');
grid on; grid minor;
xlabel('Position (m)', 'FontWeight', 'bold')
ylabel('Velocity (m/s)', 'FontWeight', 'bold')
title('Phase Portrait Comparison', 'FontWeight', 'bold')
legend('Location', 'best')
axis equal;
hold off;

sgtitle('MPC vs LQR: Trajectory Tracking Performance', 'FontSize', 16, 'FontWeight', 'bold');
set(findobj('Type','Axes'),'FontSize',12);