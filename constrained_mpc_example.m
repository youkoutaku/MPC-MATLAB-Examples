%% Constrained MPC Example
% This example demonstrates MPC with input and state constraints
% for a mass-spring-damper system.
%
% Author: Guang-Ze Yang
% Date: 2025

clear all;
close all;
clc;

%% System Definition: Mass-Spring-Damper
% Continuous-time system: m*x'' + c*x' + k*x = F
% State: [position; velocity]

m = 1.0;   % Mass (kg)
c = 0.5;   % Damping coefficient
k = 2.0;   % Spring constant

% Continuous-time state-space
Ac = [0, 1; -k/m, -c/m];
Bc = [0; 1/m];
Cc = [1, 0];
Dc = 0;

% Discretization
Ts = 0.1;  % Sampling time (s)
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
N = 15;              % Prediction horizon
Q = diag([100, 10]); % State weighting matrix
R = 1;               % Input weighting matrix
P = dare(A, B, Q, R);% Terminal cost (solution to DARE)

% Reference
r = 0.5;             % Target position

% Initial state
x0 = [0; 0];

% Simulation parameters
Tsim = 100;

% Constraints
u_min = -2.0;        % Minimum force
u_max = 2.0;         % Maximum force
x1_min = -1.0;       % Minimum position
x1_max = 2.0;        % Maximum position
x2_min = -2.0;       % Minimum velocity
x2_max = 2.0;        % Maximum velocity

%% MPC Simulation
x = x0;
x_history = zeros(nx, Tsim);
u_history = zeros(nu, Tsim);
y_history = zeros(ny, Tsim);

fprintf('Running Constrained MPC Simulation...\n');

for k = 1:Tsim
    % Solve constrained MPC problem
    [u_opt, exitflag] = solve_constrained_mpc(A, B, C, x, r, N, Q, R, P, ...
                                               u_min, u_max, x1_min, x1_max, ...
                                               x2_min, x2_max);
    
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
figure('Position', [100, 100, 1200, 800]);

% Plot position
subplot(4,1,1);
plot(0:Tsim-1, y_history, 'b-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, r*ones(1,Tsim), 'r--', 'LineWidth', 1.5);
plot(0:Tsim-1, x1_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(0:Tsim-1, x1_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('Position [m]');
legend('Position', 'Reference', 'Constraints', 'Location', 'best');
title('Constrained MPC - Position Tracking');

% Plot velocity
subplot(4,1,2);
plot(0:Tsim-1, x_history(2,:), 'g-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, x2_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(0:Tsim-1, x2_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('Velocity [m/s]');
legend('Velocity', 'Constraints', 'Location', 'best');
title('Velocity');

% Plot control input
subplot(4,1,3);
stairs(0:Tsim-1, u_history, 'r-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, u_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(0:Tsim-1, u_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('Control Force [N]');
legend('Control Input', 'Constraints', 'Location', 'best');
title('Control Input');

% Phase portrait
subplot(4,1,4);
plot(x_history(1,:), x_history(2,:), 'b-', 'LineWidth', 2);
hold on;
plot(x0(1), x0(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(r, 0, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
% Draw constraint box
rectangle('Position', [x1_min, x2_min, x1_max-x1_min, x2_max-x2_min], ...
          'EdgeColor', 'k', 'LineStyle', '--', 'LineWidth', 1);
grid on;
xlabel('Position [m]');
ylabel('Velocity [m/s]');
legend('Trajectory', 'Initial State', 'Target', 'Constraints', 'Location', 'best');
title('Phase Portrait');

%% Constrained MPC Solver Function
function [u_opt, exitflag] = solve_constrained_mpc(A, B, C, x0, r, N, Q, R, P, ...
                                                     u_min, u_max, x1_min, x1_max, ...
                                                     x2_min, x2_max)
    % Solve constrained MPC optimization problem
    %
    % Minimize: sum_{i=0}^{N-1} [(x_i - x_ref)'*Q*(x_i - x_ref) + u_i'*R*u_i] 
    %           + (x_N - x_ref)'*P*(x_N - x_ref)
    % Subject to: x_{i+1} = A*x_i + B*u_i
    %             u_min <= u_i <= u_max
    %             x_min <= x_i <= x_max
    
    nx = size(A, 1);
    nu = size(B, 2);
    
    % Reference state
    x_ref = [r; 0];
    
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
    
    X_ref = repmat(x_ref, N, 1);
    
    % Quadratic cost
    H = Su'*Q_bar*Su + R_bar;
    f = Su'*Q_bar*(Sx*x0 - X_ref);
    H = (H + H')/2;
    
    % Input constraints
    lb = u_min * ones(N*nu, 1);
    ub = u_max * ones(N*nu, 1);
    
    % State constraints: x_min <= Sx*x0 + Su*U <= x_max
    A_ineq = [Su; -Su];
    b_ineq = [repmat([x1_max; x2_max], N, 1) - Sx*x0;
              -repmat([x1_min; x2_min], N, 1) + Sx*x0];
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    [U_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
    
    if exitflag < 0
        u_opt = zeros(N*nu, 1);
    else
        u_opt = reshape(U_opt, nu, N);
    end
end
