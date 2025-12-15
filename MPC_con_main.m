%% MPC with Constraints for Trajectory Tracking
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   This script demonstrates constrained MPC for trajectory tracking
%   with input, state, and control rate constraints.
%
% Features:
%   - Input constraints (acceleration limits)
%   - State constraints (position and velocity limits)
%   - Control rate constraints (jerk limits)
%   - Time-varying sinusoidal reference trajectory
%
% Requirements:
%   - MATLAB R2020b or later
%   - Optimization Toolbox (quadprog)
%   - lib/ folder in path (QP_Transform.m)
%
% Usage:
%   addpath('lib')
%   run('MPC_con_main.m')
%----------------------------------------------------------%
clear;
close all;
clc;
addpath('lib');  % Add library folder to path
set(0, 'DefaultAxesFontName', 'Times New Roman')
set(0, 'DefaultAxesFontSize', 14)

%% System Model
A = [0 1; 0 0];
n = size(A,1);
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
Q_mpc = [1 0; 0 0.2];   % State tracking weight
Qf_mpc = 80 * [1 0; 0 0.2];  % Terminal state weight
R_mpc = 100;             % Control rate weight

%% Constraints Definition
% Input constraints (acceleration limits)
u_min = -5;    % minimum acceleration (m/s^2)
u_max = 5;     % maximum acceleration (m/s^2)

% State constraints (position and velocity limits)
x1_min = 0;    % minimum position (m)
x1_max = 20;   % maximum position (m)
x2_min = -10;   % minimum velocity (m/s)
x2_max = 10;    % maximum velocity (m/s)

% Control rate constraints (jerk limits)
du_min = -3;   % minimum control rate (m/s^3)
du_max = 3;    % maximum control rate (m/s^3)

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

% Sinusoidal trajectory (same as MPC_trajectory_main.m)
xr_traj = zeros(n, k_steps+1);
xr_traj(1,:) = 5 * sin(0.5*t_ref) + 10;  % position: amplitude=5, freq=0.5, offset=10
xr_traj(2,:) = 5 * 0.5 * cos(0.5*t_ref);  % velocity: derivative of position

%% MPC Setup with Constraints
% Use QP_Transform function to build prediction matrices
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

%% Build Constraint Matrices for QP
% Constraints are on ΔU: [Δu(k); Δu(k+1); ...; Δu(k+Np-1)]
% We need to formulate: A_ineq * ΔU <= b_ineq

% 1. Control rate constraints: du_min <= Δu(k+i) <= du_max
A_du = [eye(Np*p); -eye(Np*p)];
b_du_template = [du_max * ones(Np*p, 1); -du_min * ones(Np*p, 1)];

% 2. Input constraints will be time-varying (depends on u(k-1))
% u_min <= u(k-1) + sum(Δu) <= u_max
% Build cumulative sum matrix
T_sum = tril(ones(Np, Np));  % Lower triangular matrix for cumulative sum
T_sum = kron(T_sum, eye(p));  % Expand to input dimension

%% Simulation - discrete time system
options = optimoptions('quadprog', 'Display', 'off');

% Statistics for constraint violations
constraint_active = zeros(4, k_steps);  % Track which constraints are active
feasibility_status = ones(k_steps, 1);  % Track feasibility at each step

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
            Xr((i-1)*n+1:i*n) = xr_traj(:, end);
        end
    end
    
    % Compute the linear term: f = Su'*Qp*(Sx*x(k) + Iu*u_prev - Xr)
    f = Su'*Qp*(Sx*x_mpc(:,k) + Iu*u_prev - Xr);
    
    %% Build time-varying constraints
    % Input constraints: u_min <= u(k-1) + T_sum*ΔU <= u_max
    A_u = [T_sum; -T_sum];
    b_u = [u_max * ones(Np*p, 1) - u_prev; 
           -u_min * ones(Np*p, 1) + u_prev];
    
    % State constraints: x_min <= Sx*x(k) + Su*ΔU + Iu*u_prev <= x_max
    % Extract position and velocity constraints separately
    % Position: x1_min <= x1(k+i) <= x1_max
    % Velocity: x2_min <= x2(k+i) <= x2_max
    C_x = zeros(2*Np, Np*n);
    for i = 1:Np
        C_x(i, (i-1)*n+1) = 1;           % Select position at step i
        C_x(Np+i, (i-1)*n+2) = 1;        % Select velocity at step i
    end
    
    Sx_x_plus_Iu_u = Sx*x_mpc(:,k) + Iu*u_prev;
    A_x = [C_x*Su; -C_x*Su];
    b_x = [x1_max * ones(Np, 1) - C_x(1:Np, :)*Sx_x_plus_Iu_u;
           x2_max * ones(Np, 1) - C_x(Np+1:2*Np, :)*Sx_x_plus_Iu_u;
           -x1_min * ones(Np, 1) + C_x(1:Np, :)*Sx_x_plus_Iu_u;
           -x2_min * ones(Np, 1) + C_x(Np+1:2*Np, :)*Sx_x_plus_Iu_u];
    
    % Combine all constraints
    A_ineq = [A_du; A_u; A_x];
    b_ineq = [b_du_template; b_u; b_x];
    
    % Solve QP with constraints: min 0.5*ΔU'*H*ΔU + f'*ΔU s.t. A_ineq*ΔU <= b_ineq
    [DU, ~, exitflag, ~, lambda] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
    
    if exitflag ~= 1
        warning('Quadprog did not converge at step %d, exitflag = %d', k, exitflag);
        feasibility_status(k) = 0;
        % Use zero control increment if optimization fails
        du = 0;
        u_mpc(:,k) = u_prev;
        x_mpc(:,k+1) = A * x_mpc(:,k) + B * u_mpc(:,k) + d(:,k);
        continue;
    end
    
    % Check which constraints are active
    active_tol = 1e-6;
    constraint_active(1, k) = any(abs(A_du*DU - b_du_template) < active_tol);      % Rate constraint
    constraint_active(2, k) = any(abs(A_u*DU - b_u) < active_tol);                  % Input constraint
    constraint_active(3, k) = any(abs(A_x*DU - b_x) < active_tol);                  % State constraint
    
    % Extract the first control increment
    du = DU(1:p);
    
    % Compute control input: u(k) = u(k-1) + Δu(k)
    u_mpc(:,k) = u_prev + du;
    
    % System update
    x_mpc(:,k+1) = A * x_mpc(:,k) + B * u_mpc(:,k) + d(:,k);
end

%% Performance Analysis
fprintf('\n========================================\n');
fprintf('   MPC with Constraints Performance    \n');
fprintf('========================================\n');
fprintf('Simulation completed successfully!\n');
fprintf('Feasible solutions: %d/%d steps (%.1f%%)\n', sum(feasibility_status), k_steps, ...
    100*sum(feasibility_status)/k_steps);
fprintf('\nConstraint Activity:\n');
fprintf('  Control rate constraints active: %d/%d steps (%.1f%%)\n', ...
    sum(constraint_active(1,:)), k_steps, 100*sum(constraint_active(1,:))/k_steps);
fprintf('  Input constraints active: %d/%d steps (%.1f%%)\n', ...
    sum(constraint_active(2,:)), k_steps, 100*sum(constraint_active(2,:))/k_steps);
fprintf('  State constraints active: %d/%d steps (%.1f%%)\n', ...
    sum(constraint_active(3,:)), k_steps, 100*sum(constraint_active(3,:))/k_steps);

% Create time vectors
t_state = 0:ts:Time;
t_input = 0:ts:Time-ts;

% Calculate tracking errors
error_pos = x_mpc(1,:) - xr_traj(1,:);
error_vel = x_mpc(2,:) - xr_traj(2,:);

% Performance metrics
RMSE_pos = sqrt(mean(error_pos.^2));
RMSE_vel = sqrt(mean(error_vel.^2));
Max_error_pos = max(abs(error_pos));
Max_error_vel = max(abs(error_vel));

fprintf('\nTracking Performance:\n');
fprintf('  Position RMSE: %.4f m, Max Error: %.4f m\n', RMSE_pos, Max_error_pos);
fprintf('  Velocity RMSE: %.4f m/s, Max Error: %.4f m/s\n', RMSE_vel, Max_error_vel);

% Check constraint violations
pos_violations = sum(x_mpc(1,:) < x1_min | x_mpc(1,:) > x1_max);
vel_violations = sum(x_mpc(2,:) < x2_min | x_mpc(2,:) > x2_max);
u_violations = sum(u_mpc(1,:) < u_min | u_mpc(1,:) > u_max);

fprintf('\nConstraint Violations:\n');
fprintf('  Position: %d violations\n', pos_violations);
fprintf('  Velocity: %d violations\n', vel_violations);
fprintf('  Input: %d violations\n', u_violations);

%% Plot Results
figure('Position', [50, 50, 1400, 800], 'Color', 'w', 'Name', 'MPC with Constraints Analysis')

% Define color scheme
color_ref = 'r';  % Red for reference
color_mpc = 'b';  % Blue for MPC
color_const = [0.5 0.5 0.5];   % Gray for constraints
color_error_pos = [0.93 0.29 0.29];  % Red for position error
color_error_vel = [0.28 0.68 0.93];  % Light blue for velocity error

% Position tracking with constraints
subplot(3, 2, 1);
hold on;
% Shade constraint violation zones (if any)
patch([0 Time Time 0], [x1_max x1_max max(ylim)*1.2 max(ylim)*1.2], ...
    color_const, 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility', 'off');
patch([0 Time Time 0], [min(ylim)*1.2 min(ylim)*1.2 x1_min x1_min], ...
    color_const, 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(t_state, xr_traj(1,:), '--', 'Color', color_ref, 'LineWidth', 2.5, 'DisplayName', 'Reference');
plot(t_state, x_mpc(1,:), '-', 'Color', color_mpc, 'LineWidth', 2.5, 'DisplayName', 'MPC');
yline(x1_max, ':', 'Color', color_const, 'LineWidth', 2, 'DisplayName', 'Constraints');
yline(x1_min, ':', 'Color', color_const, 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Position (m)', 'FontWeight', 'bold')
legend('Location', 'northeast', 'FontSize', 10)
title('Position Tracking', 'FontSize', 12, 'FontWeight', 'bold')
text(0.98, 0.02, sprintf('RMSE: %.3f m\nMax Err: %.3f m', RMSE_pos, Max_error_pos), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'BackgroundColor', [1 1 1 0.8], 'EdgeColor', 'k', 'FontSize', 9, 'Margin', 3);
xlim([0 Time])
hold off;

% Velocity tracking with constraints
subplot(3, 2, 3);
hold on;
% Shade constraint violation zones
yl = ylim;
patch([0 Time Time 0], [x2_max x2_max yl(2)*1.1 yl(2)*1.1], ...
    color_const, 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility', 'off');
patch([0 Time Time 0], [yl(1)*1.1 yl(1)*1.1 x2_min x2_min], ...
    color_const, 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(t_state, xr_traj(2,:), '--', 'Color', color_ref, 'LineWidth', 2.5, 'DisplayName', 'Reference');
plot(t_state, x_mpc(2,:), '-', 'Color', color_mpc, 'LineWidth', 2.5, 'DisplayName', 'MPC');
yline(x2_max, ':', 'Color', color_const, 'LineWidth', 2, 'DisplayName', 'Constraints');
yline(x2_min, ':', 'Color', color_const, 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Velocity (m/s)', 'FontWeight', 'bold')
legend('Location', 'northeast', 'FontSize', 10)
title('Velocity Tracking', 'FontSize', 12, 'FontWeight', 'bold')
text(0.98, 0.02, sprintf('RMSE: %.3f m/s\nMax Err: %.3f m/s', RMSE_vel, Max_error_vel), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'BackgroundColor', [1 1 1 0.8], 'EdgeColor', 'k', 'FontSize', 9, 'Margin', 3);
xlim([0 Time])
hold off;

% Control input with constraints
subplot(3, 2, 5);
hold on;
% Shade constraint zones
yl = ylim;
patch([0 Time Time 0], [u_max u_max max(yl(2), u_max*1.5) max(yl(2), u_max*1.5)], ...
    [1 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
patch([0 Time Time 0], [min(yl(1), u_min*1.5) min(yl(1), u_min*1.5) u_min u_min], ...
    [1 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
stairs(t_input, u_mpc(1,:), '-', 'Color', color_mpc, 'LineWidth', 2.5, 'DisplayName', 'Control Input');
yline(u_max, '-.', 'Color', [0.8 0.2 0.2], 'LineWidth', 2, 'DisplayName', 'Input Limits');
yline(u_min, '-.', 'Color', [0.8 0.2 0.2], 'LineWidth', 2, 'HandleVisibility', 'off');
yline(0, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1, 'HandleVisibility', 'off');
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Acceleration (m/s²)', 'FontWeight', 'bold')
title('Control Input', 'FontSize', 12, 'FontWeight', 'bold')
legend('Location', 'best', 'FontSize', 10)
xlim([0 Time])
ylim([min(u_min*1.2, min(u_mpc)-0.5), max(u_max*1.2, max(u_mpc)+0.5)])
hold off;

% Position error
subplot(3, 2, 2);
hold on;
% Error envelope
fill([t_state, fliplr(t_state)], [error_pos, zeros(size(error_pos))], ...
    color_error_pos, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'DisplayName', 'Error Envelope');
% Error line
plot(t_state, error_pos, '-', 'Color', color_error_pos, 'LineWidth', 2.5, 'DisplayName', 'Position Error');
yline(0, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off');
% Mark maximum error
[max_err, max_idx] = max(abs(error_pos));
plot(t_state(max_idx), error_pos(max_idx), 'p', 'MarkerSize', 12, ...
    'MarkerFaceColor', [1 0.3 0.3], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Max: %.3f m', max_err));
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Error (m)', 'FontWeight', 'bold')
title('Position Tracking Error', 'FontSize', 12, 'FontWeight', 'bold')
legend('Location', 'best', 'FontSize', 9)
xlim([0 Time])
hold off;

% Velocity error
subplot(3, 2, 4);
hold on;
% Error envelope
fill([t_state, fliplr(t_state)], [error_vel, zeros(size(error_vel))], ...
    color_error_vel, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'DisplayName', 'Error Envelope');
% Error line
plot(t_state, error_vel, '-', 'Color', color_error_vel, 'LineWidth', 2.5, 'DisplayName', 'Velocity Error');
yline(0, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off');
% Mark maximum error
[max_err, max_idx] = max(abs(error_vel));
plot(t_state(max_idx), error_vel(max_idx), 'p', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0.3 0.7 1], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Max: %.3f m/s', max_err));
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Error (m/s)', 'FontWeight', 'bold')
title('Velocity Tracking Error', 'FontSize', 12, 'FontWeight', 'bold')
legend('Location', 'best', 'FontSize', 9)
xlim([0 Time])
hold off;

% Phase portrait with constraints
subplot(3, 2, 6);
hold on;
% Draw constraint box with fill
rectangle('Position', [x1_min, x2_min, x1_max-x1_min, x2_max-x2_min], ...
    'FaceColor', [0.95 0.95 0.95], 'EdgeColor', color_const, 'LineStyle', '--', ...
    'LineWidth', 2.5);
% Add invisible patch for legend
patch(NaN, NaN, [0.95 0.95 0.95], 'EdgeColor', color_const, 'LineStyle', '--', ...
    'LineWidth', 2.5, 'DisplayName', 'Feasible Region');
% Trajectories
plot(xr_traj(1,:), xr_traj(2,:), '--', 'Color', color_ref, 'LineWidth', 2.5, 'DisplayName', 'Reference');
plot(x_mpc(1,:), x_mpc(2,:), '-', 'Color', color_mpc, 'LineWidth', 2.5, 'DisplayName', 'MPC');
% Direction arrows
arrow_idx = round(linspace(1, length(x_mpc), 8));
quiver(x_mpc(1,arrow_idx(2:end-1)), x_mpc(2,arrow_idx(2:end-1)), ...
    diff(x_mpc(1,arrow_idx(1:end-1))), diff(x_mpc(2,arrow_idx(1:end-1))), 0.5, ...
    'Color', color_mpc, 'LineWidth', 1.5, 'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
% Start and end markers
plot(x_mpc(1,1), x_mpc(2,1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.2 0.8 0.2], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Start');
plot(x_mpc(1,end), x_mpc(2,end), 's', 'MarkerSize', 12, 'MarkerFaceColor', [0.9 0.2 0.2], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'End');
grid on; grid minor;
xlabel('Position (m)', 'FontWeight', 'bold')
ylabel('Velocity (m/s)', 'FontWeight', 'bold')
title('Phase Portrait', 'FontSize', 12, 'FontWeight', 'bold')
legend('Location', 'best', 'FontSize', 9)
axis equal;
xlim([min(x1_min-1, min([xr_traj(1,:), x_mpc(1,:)])-1), max(x1_max+1, max([xr_traj(1,:), x_mpc(1,:)])+1)])
ylim([min(x2_min-0.5, min([xr_traj(2,:), x_mpc(2,:)])-0.5), max(x2_max+0.5, max([xr_traj(2,:), x_mpc(2,:)])+0.5)])
hold off;

% Add overall title with performance summary
sgtitle({'\bf MPC Trajectory Tracking with Constraints', ...
    sprintf('\\rm Performance: Pos RMSE=%.3f m, Vel RMSE=%.3f m/s | Feasibility: %d/%d steps', ...
    RMSE_pos, RMSE_vel, sum(feasibility_status), k_steps)}, ...
    'FontSize', 14);

% Adjust subplot spacing
set(findobj('Type','Axes'),'FontSize',11, 'LineWidth', 1.2);
set(gcf, 'DefaultAxesFontName', 'Times New Roman');
