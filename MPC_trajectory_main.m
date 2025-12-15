%% MPC for Trajectory Tracking Problem with no Constraints
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   Basic MPC implementation for time-varying reference tracking.
%   This is the recommended starting point for learning MPC.
%
% Features:
%   - Unconstrained MPC formulation
%   - Sinusoidal reference trajectory
%   - QP-based optimization
%   - Performance metrics (RMSE, max error)
%
% Requirements:
%   - MATLAB R2020b or later
%   - Optimization Toolbox (quadprog)
%   - lib/ folder in path (QP_Transform.m)
%
% Usage:
%   addpath('lib')
%   run('MPC_trajectory_main.m')
%----------------------------------------------------------%
clear;
close all;
clc;
addpath('lib');  % Add library folder to path
set(0, 'DefaultAxesFontName', 'Times New Roman')
set(0, 'DefaultAxesFontSize', 14)

%% System Model
A = [0 1; 0 0];
n= size(A,1);
B = [0; 1];
p = size(B,2);
C = [1 0; 0 1];
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
Qf_mpc= 50 * [1 0; 0 0.2]; % 100, 10, 1
R_mpc = 10; % 1, 100

%% Initialization
% the number of the prediction horizon
Np = 30;
% the state of the system
x0 = [0; 0];
% the state of the system by MPC
x_mpc = zeros(n, k_steps+1);
x_mpc(:, 1) = x0;
% the input by MPC
u_mpc = zeros(p, k_steps);

d = zeros(n, k_steps); % disturbance

%% Generate time-varying reference trajectory
t_ref = 0:ts:Time;  % time vector

% Define reference trajectory (you can change this to any function)
% Option 1: Sinusoidal trajectory
xr_traj = zeros(n, k_steps+1);
xr_traj(1,:) = 5 * sin(0.5*t_ref) + 10;  % position: amplitude=5, freq=0.5, offset=10
xr_traj(2,:) = 5 * 0.5 * cos(0.5*t_ref);  % velocity: derivative of position

% Option 2: Ramp trajectory (uncomment to use)
% xr_traj(1,:) = 0.5 * t_ref + 5;  % position: linearly increasing
% xr_traj(2,:) = 0.5 * ones(1, k_steps+1);  % velocity: constant

% Option 3: Step changes (uncomment to use)
% xr_traj(1,:) = 10 * (t_ref >= 3) + 5;  % step at t=3s
% xr_traj(2,:) = zeros(1, k_steps+1);  % zero velocity

%% MPC Setup with Direct Tracking Formulation
% Cost: sum ||x(k+i) - xr(k+i)||_Q^2 + ||Δu(k+i)||_R^2
% Prediction: X = Sx*x(k) + Su*ΔU + Iu*u(k-1)

% Use QP_Transform function to build prediction matrices
[Sx, Su, Qp, Rp, ~, H] = QP_Transform(A, B, Q_mpc, R_mpc, Qf_mpc, Np);
% Note: QP_Transform returns [Ap, Bp, Qp, Rp, F, H]
% where Ap=Sx (state prediction), Bp=Su (input prediction)

% Build Iu matrix (effect of u(k-1) on future states)
% This accounts for the cumulative effect of previous control input
Iu = zeros(Np*n, p);
for i = 1:Np
    temp = zeros(n, p);
    for j = 0:i-1
        temp = temp + A^j * B;
    end
    Iu((i-1)*n+1:i*n, :) = temp;
end

% Ensure H is symmetric (numerical stability)
H = (H + H')/2;

%%  Simulation - discrete time system
options = optimoptions('quadprog', 'Display', 'off');

for k = 1 : k_steps
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
            % If beyond simulation time, use last reference
            Xr((i-1)*n+1:i*n) = xr_traj(:, end);
        end
    end
    
    % Compute the linear term: f = Su'*Qp*(Sx*x(k) + Iu*u_prev - Xr)
    f = Su'*Qp*(Sx*x_mpc(:,k) + Iu*u_prev - Xr);
    
    % Solve QP: min 0.5*ΔU'*H*ΔU + f'*ΔU
    [DU, ~, exitflag] = quadprog(H, f, [], [], [], [], [], [], [], options);
    
    if exitflag ~= 1
        warning('Quadprog did not converge at step %d', k);
    end
    
    % Extract the first control increment
    du = DU(1:p);
    
    % Compute control input: u(k) = u(k-1) + Δu(k)
    u_mpc(:,k) = u_prev + du;
    
    % System update
    x_mpc(:,k+1) = A * x_mpc(:,k) + B * u_mpc(:,k) + d(:,k);
end

%% Plot
% Create time vectors
t_state = 0:ts:Time;  % for state (k_steps+1 points)
t_input = 0:ts:Time-ts;  % for input (k_steps points)

% Calculate tracking errors
error_pos = x_mpc(1,:) - xr_traj(1,:);
error_vel = x_mpc(2,:) - xr_traj(2,:);

% Calculate performance metrics
RMSE_pos = sqrt(mean(error_pos.^2));
RMSE_vel = sqrt(mean(error_vel.^2));
Max_error_pos = max(abs(error_pos));
Max_error_vel = max(abs(error_vel));

% Create figure with two columns
figure('Position', [50, 50, 1400, 900], 'Color', 'w')

% Left column: Tracking results
% Position tracking
subplot(3, 2, 1);
hold on;
plot(t_state, xr_traj(1,:), 'r--', 'LineWidth', 2);
plot(t_state, x_mpc(1,:), 'b-', 'LineWidth', 2.5);
grid on; grid minor;
xlabel('Time (s)')
ylabel('Position (m)')
legend('Reference', 'MPC', 'Location', 'best')
title('Position Tracking')
text(0.02, 0.98, sprintf('RMSE: %.3f m\nMax Error: %.3f m', RMSE_pos, Max_error_pos), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'BackgroundColor', 'w', 'FontSize', 10);
hold off;

% Velocity tracking
subplot(3, 2, 3);
hold on;
plot(t_state, xr_traj(2,:), 'r--', 'LineWidth', 2);
plot(t_state, x_mpc(2,:), 'b-', 'LineWidth', 2.5);
grid on; grid minor;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Reference', 'MPC', 'Location', 'best')
title('Velocity Tracking')
text(0.02, 0.98, sprintf('RMSE: %.3f m/s\nMax Error: %.3f m/s', RMSE_vel, Max_error_vel), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'BackgroundColor', 'w', 'FontSize', 10);
hold off;

% Control input (acceleration)
subplot(3, 2, 5);
hold on;
stairs(t_input, u_mpc(1,:), 'b-', 'LineWidth', 2.5);
yline(0, 'k--', 'LineWidth', 1);
grid on; grid minor;
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Control Input (Acceleration)')
xlim([0 Time])
hold off;

% Right column: Error analysis and phase portrait
% Position error
subplot(3, 2, 2);
hold on;
plot(t_state, error_pos, 'Color', [0.8 0.2 0.2], 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
fill([t_state, fliplr(t_state)], [error_pos, zeros(size(error_pos))], ...
    [0.8 0.2 0.2], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
grid on; grid minor;
xlabel('Time (s)')
ylabel('Error (m)')
title('Position Tracking Error')
hold off;

% Velocity error
subplot(3, 2, 4);
hold on;
plot(t_state, error_vel, 'Color', [0.2 0.6 0.8], 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
fill([t_state, fliplr(t_state)], [error_vel, zeros(size(error_vel))], ...
    [0.2 0.6 0.8], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
grid on; grid minor;
xlabel('Time (s)')
ylabel('Error (m/s)')
title('Velocity Tracking Error')
hold off;

% Phase portrait (Position vs Velocity)
subplot(3, 2, 6);
hold on;
plot(xr_traj(1,:), xr_traj(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
plot(x_mpc(1,:), x_mpc(2,:), 'b-', 'LineWidth', 2.5, 'DisplayName', 'MPC');
plot(x_mpc(1,1), x_mpc(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(x_mpc(1,end), x_mpc(2,end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
grid on; grid minor;
xlabel('Position (m)')
ylabel('Velocity (m/s)')
title('Phase Portrait (Position-Velocity Plane)')
legend('Location', 'best')
axis equal;
hold off;

% Add overall title
sgtitle('MPC Trajectory Tracking Performance', 'FontSize', 16, 'FontWeight', 'bold');

set(findobj('Type','Axes'),'FontSize',12);