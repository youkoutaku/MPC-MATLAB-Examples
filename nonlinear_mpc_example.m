%% Nonlinear MPC Example
% This example demonstrates nonlinear MPC using Sequential Quadratic Programming (SQP)
% for a Van der Pol oscillator system
%
% Author: Guang-Ze Yang
% Date: 2025

clear all;
close all;
clc;

%% System Definition: Van der Pol Oscillator
% Nonlinear dynamics: x1' = x2
%                     x2' = mu*(1-x1^2)*x2 - x1 + u
% where mu is the nonlinearity parameter

mu = 1.0;  % Nonlinearity parameter

% Discretization
Ts = 0.1;  % Sampling time

% System dimensions
nx = 2;    % Number of states
nu = 1;    % Number of inputs

%% MPC Parameters
N = 10;              % Prediction horizon
Q = diag([10, 1]);   % State weighting
R = 0.5;             % Input weighting
P = Q;               % Terminal cost

% Reference state
x_ref = [0; 0];      % Origin stabilization

% Initial state
x0 = [2; 0];

% Simulation parameters
Tsim = 80;

% Constraints
u_min = -3.0;
u_max = 3.0;
x1_min = -3.0;
x1_max = 3.0;
x2_min = -3.0;
x2_max = 3.0;

%% Nonlinear MPC Simulation
x = x0;
x_history = zeros(nx, Tsim);
u_history = zeros(nu, Tsim);

fprintf('Running Nonlinear MPC Simulation...\n');

% Initial guess for optimization
u_init = zeros(N*nu, 1);

for k = 1:Tsim
    % Solve nonlinear MPC problem
    [u_opt, exitflag] = solve_nonlinear_mpc(x, x_ref, N, Q, R, P, ...
                                            u_min, u_max, x1_min, x1_max, ...
                                            x2_min, x2_max, mu, Ts, u_init);
    
    if exitflag <= 0
        warning('Nonlinear MPC solver failed at step %d', k);
        u = 0;
    else
        u = u_opt(1);
        % Warm start for next iteration
        u_init = [u_opt(2:end); u_opt(end)];
    end
    
    % Apply control and simulate nonlinear system
    x = simulate_vanderpol(x, u, mu, Ts);
    
    % Store history
    x_history(:, k) = x;
    u_history(:, k) = u;
end

fprintf('Simulation complete!\n');

%% Visualization
figure('Position', [100, 100, 1200, 800]);

% Plot state x1
subplot(3,1,1);
plot(0:Tsim-1, x_history(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, x_ref(1)*ones(1,Tsim), 'r--', 'LineWidth', 1.5);
plot(0:Tsim-1, x1_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(0:Tsim-1, x1_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('State x_1');
legend('x_1', 'Reference', 'Constraints', 'Location', 'best');
title('Nonlinear MPC - Van der Pol Oscillator');

% Plot state x2
subplot(3,1,2);
plot(0:Tsim-1, x_history(2,:), 'g-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, x_ref(2)*ones(1,Tsim), 'r--', 'LineWidth', 1.5);
plot(0:Tsim-1, x2_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(0:Tsim-1, x2_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('State x_2');
legend('x_2', 'Reference', 'Constraints', 'Location', 'best');

% Plot control input
subplot(3,1,3);
stairs(0:Tsim-1, u_history, 'r-', 'LineWidth', 2);
hold on;
plot(0:Tsim-1, u_max*ones(1,Tsim), 'k--', 'LineWidth', 1);
plot(0:Tsim-1, u_min*ones(1,Tsim), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time Step');
ylabel('Control Input u');
legend('Control Input', 'Constraints', 'Location', 'best');

% Phase portrait
figure('Position', [150, 150, 600, 600]);
plot(x_history(1,:), x_history(2,:), 'b-', 'LineWidth', 2);
hold on;
plot(x0(1), x0(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(x_ref(1), x_ref(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
% Draw constraint box
rectangle('Position', [x1_min, x2_min, x1_max-x1_min, x2_max-x2_min], ...
          'EdgeColor', 'k', 'LineStyle', '--', 'LineWidth', 1.5);
grid on;
xlabel('State x_1');
ylabel('State x_2');
legend('Trajectory', 'Initial State', 'Target', 'Constraints', 'Location', 'best');
title('Phase Portrait - Van der Pol Oscillator with NMPC');
axis equal;

%% Helper Functions

function [u_opt, exitflag] = solve_nonlinear_mpc(x0, x_ref, N, Q, R, P, ...
                                                  u_min, u_max, x1_min, x1_max, ...
                                                  x2_min, x2_max, mu, Ts, u_init)
    % Solve nonlinear MPC using fmincon (SQP)
    
    nx = length(x0);
    nu = 1;
    
    % Cost function
    cost_fun = @(U) compute_cost(U, x0, x_ref, N, Q, R, P, mu, Ts);
    
    % Nonlinear constraints (dynamics are embedded in cost via simulation)
    nonlcon = @(U) compute_constraints(U, x0, N, x1_min, x1_max, x2_min, x2_max, mu, Ts);
    
    % Input bounds
    lb = u_min * ones(N*nu, 1);
    ub = u_max * ones(N*nu, 1);
    
    % Solve using fmincon
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
                          'MaxIterations', 100);
    [U_opt, ~, exitflag] = fmincon(cost_fun, u_init, [], [], [], [], lb, ub, ...
                                   nonlcon, options);
    
    u_opt = U_opt;
end

function cost = compute_cost(U, x0, x_ref, N, Q, R, P, mu, Ts)
    % Compute the MPC cost function
    
    cost = 0;
    x = x0;
    
    for i = 1:N
        u = U(i);
        
        % State cost
        if i < N
            cost = cost + (x - x_ref)' * Q * (x - x_ref) + u' * R * u;
        else
            % Terminal cost
            cost = cost + (x - x_ref)' * P * (x - x_ref);
        end
        
        % Simulate one step
        x = simulate_vanderpol(x, u, mu, Ts);
    end
end

function [c, ceq] = compute_constraints(U, x0, N, x1_min, x1_max, x2_min, x2_max, mu, Ts)
    % Compute nonlinear constraints
    % We enforce state constraints along the prediction horizon
    
    nx = length(x0);
    x = x0;
    
    % Inequality constraints: c <= 0
    c = zeros(N*4, 1);
    
    for i = 1:N
        u = U(i);
        x = simulate_vanderpol(x, u, mu, Ts);
        
        % State constraints
        c((i-1)*4+1) = x(1) - x1_max;  % x1 <= x1_max
        c((i-1)*4+2) = x1_min - x(1);  % x1 >= x1_min
        c((i-1)*4+3) = x(2) - x2_max;  % x2 <= x2_max
        c((i-1)*4+4) = x2_min - x(2);  % x2 >= x2_min
    end
    
    % Equality constraints (none in this case)
    ceq = [];
end

function x_next = simulate_vanderpol(x, u, mu, Ts)
    % Simulate Van der Pol oscillator using RK4 integration
    
    % Define continuous-time dynamics
    f = @(x, u) [x(2); 
                 mu*(1-x(1)^2)*x(2) - x(1) + u];
    
    % RK4 integration
    k1 = f(x, u);
    k2 = f(x + Ts/2*k1, u);
    k3 = f(x + Ts/2*k2, u);
    k4 = f(x + Ts*k3, u);
    
    x_next = x + Ts/6*(k1 + 2*k2 + 2*k3 + k4);
end
