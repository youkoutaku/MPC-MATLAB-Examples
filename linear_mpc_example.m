%% Linear MPC Example
% This example demonstrates a basic Model Predictive Control (MPC) 
% implementation for a linear discrete-time system.
%
% System: x(k+1) = Ax(k) + Bu(k)
%         y(k) = Cx(k)
%
% Author: Guang-Ze Yang
% Date: 2025

clear all;
close all;
clc;

%% System Definition
% Define a simple second-order system
% Example: Double integrator system
A = [1 0.1; 0 1];  % State transition matrix
B = [0.005; 0.1];  % Input matrix
C = [1 0];         % Output matrix
D = 0;             % Feedthrough matrix

% System dimensions
nx = size(A, 1);   % Number of states
nu = size(B, 2);   % Number of inputs
ny = size(C, 1);   % Number of outputs

%% MPC Parameters
N = 10;            % Prediction horizon
Q = diag([10, 1]); % State weighting matrix
R = 0.1;           % Input weighting matrix
P = Q;             % Terminal cost matrix

% Reference trajectory
r = 1.0;           % Target setpoint

% Initial state
x0 = [0; 0];

% Simulation parameters
Tsim = 50;         % Simulation time steps

% Constraints (optional)
u_min = -5;
u_max = 5;

%% MPC Simulation
x = x0;
x_history = zeros(nx, Tsim);
u_history = zeros(nu, Tsim);
y_history = zeros(ny, Tsim);

fprintf('Running Linear MPC Simulation...\n');

for k = 1:Tsim
    % Solve MPC optimization problem
    u_opt = solve_linear_mpc(A, B, C, x, r, N, Q, R, P, u_min, u_max);
    
    % Apply first control input
    u = u_opt(1);
    
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
figure('Position', [100, 100, 1000, 600]);

% Plot output
subplot(3,1,1);
plot(1:Tsim, y_history, 'b-', 'LineWidth', 2);
hold on;
plot(1:Tsim, r*ones(1,Tsim), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time Step');
ylabel('Output y');
legend('System Output', 'Reference', 'Location', 'best');
title('Linear MPC - Output Tracking');

% Plot states
subplot(3,1,2);
plot(1:Tsim, x_history(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(1:Tsim, x_history(2,:), 'g-', 'LineWidth', 2);
grid on;
xlabel('Time Step');
ylabel('States');
legend('State x_1', 'State x_2', 'Location', 'best');
title('State Trajectories');

% Plot control input
subplot(3,1,3);
stairs(1:Tsim, u_history, 'r-', 'LineWidth', 2);
hold on;
plot(1:Tsim, u_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(1:Tsim, u_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('Control Input u');
legend('Control Input', 'Constraints', 'Location', 'best');
title('Control Input');

%% MPC Solver Function
function u_opt = solve_linear_mpc(A, B, C, x0, r, N, Q, R, P, u_min, u_max)
    % Solve the MPC optimization problem
    %
    % Minimize: sum_{i=0}^{N-1} [(x_i - x_ref)'*Q*(x_i - x_ref) + u_i'*R*u_i] 
    %           + (x_N - x_ref)'*P*(x_N - x_ref)
    % Subject to: x_{i+1} = A*x_i + B*u_i
    %             u_min <= u_i <= u_max
    
    nx = size(A, 1);
    nu = size(B, 2);
    
    % Reference state (steady-state for setpoint r)
    x_ref = [r; 0];
    
    % Build optimization matrices
    % Decision variable: U = [u_0; u_1; ...; u_{N-1}]
    
    % Compute state predictions as function of U
    % x_i = A^i*x0 + sum_{j=0}^{i-1} A^{i-1-j}*B*u_j
    
    % Build lifted system matrices
    Sx = zeros(nx*N, nx);      % State prediction from x0
    Su = zeros(nx*N, nu*N);    % State prediction from U
    
    for i = 1:N
        % Sx block
        Sx((i-1)*nx+1:i*nx, :) = A^i;
        
        % Su block
        for j = 1:i
            Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j)*B;
        end
    end
    
    % Build cost function matrices
    Q_bar = kron(eye(N-1), Q);
    Q_bar = blkdiag(Q_bar, P);  % Terminal cost
    R_bar = kron(eye(N), R);
    
    % Reference trajectory
    X_ref = repmat(x_ref, N, 1);
    
    % Quadratic cost: 0.5*U'*H*U + f'*U
    H = Su'*Q_bar*Su + R_bar;
    f = Su'*Q_bar*(Sx*x0 - X_ref);
    
    % Make H symmetric (for numerical stability)
    H = (H + H')/2;
    
    % Constraints
    lb = u_min * ones(N*nu, 1);
    ub = u_max * ones(N*nu, 1);
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    U_opt = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    
    % Extract control sequence
    u_opt = reshape(U_opt, nu, N);
end
