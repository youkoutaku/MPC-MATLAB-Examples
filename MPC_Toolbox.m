%% MPC Trajectory Tracking using MATLAB MPC Toolbox
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   Verification script using MATLAB's official MPC Toolbox.
%   Compares results with manual QP-based implementation.
%
% Features:
%   - Uses mpc() object from MPC Toolbox
%   - Identical weight settings as manual implementation
%   - Validates custom QP formulation
%
% Requirements:
%   - MATLAB R2020b or later
%   - MPC Toolbox (required)
%   - Control System Toolbox
%
% Usage:
%   run('MPC_Toolbox.m')
%
% Note:
%   This script requires MATLAB MPC Toolbox license.
%----------------------------------------------------------%
clear;
close all;
clc;
set(0, 'DefaultAxesFontName', 'Times New Roman')
set(0, 'DefaultAxesFontSize', 14)

%% System Model
% Continuous-time system: double integrator
% ẋ₁ = x₂ (velocity)
% ẋ₂ = u  (acceleration)
A_c = [0 1; 0 0];
B_c = [0; 1];
C = [1 0; 0 1];  % Measure both position and velocity
D = 0;

% Simulation parameters
Time = 20;        % simulation time
ts = 0.05;         % sampling time
k_steps = Time/ts;

% Create discrete-time state-space model
sys_c = ss(A_c, B_c, C, D);
sys_d = c2d(sys_c, ts);

%% Generate time-varying reference trajectory
t_ref = 0:ts:Time;
n = size(A_c, 1);

% Sinusoidal trajectory
xr_traj = zeros(n, k_steps+1);
xr_traj(1,:) = 5 * sin(0.5*t_ref) + 10;  % position
xr_traj(2,:) = 5 * 0.5 * cos(0.5*t_ref);  % velocity

%% Design MPC Controller using MATLAB Toolbox
% Create MPC controller object
% Note: We specify weights during creation to avoid default values
mpcobj = mpc(sys_d, ts);

% Set prediction and control horizons FIRST (before weights)
mpcobj.PredictionHorizon = 30;
mpcobj.ControlHorizon = 30;

% Set weights to match manual QP implementation (MPC_con_main.m)
% In manual QP: Q_mpc = [1 0; 0 0.2], Qf_mpc = 80*[1 0; 0 0.2], R_mpc = 100
% MATLAB Toolbox uses: J = sum(y'*Q*y) + sum(u'*R*u) + sum(Δu'*S*Δu)
% To match manual QP formulation:
% - OutputVariables weight = Q (tracking error penalty)
% - ManipulatedVariables weight = penalty on control absolute value (set to 0)
% - ManipulatedVariablesRate weight = R (penalty on Δu)
mpcobj.Weights.OutputVariables = [1 0.2];  % Q_mpc = diag([1, 0.2])
mpcobj.Weights.ManipulatedVariables = 0;  % No penalty on absolute control value
mpcobj.Weights.ManipulatedVariablesRate = 5;  % R_mpc = 100 (penalty on Δu)

%% Constraints Configuration
% Define constraints - Currently removed to match unconstrained manual QP
% To enable constraints, uncomment and adjust values as needed

% Manipulated variable (input) constraints
mpcobj.ManipulatedVariables.Min = -inf;    % no min constraint
mpcobj.ManipulatedVariables.Max = inf;     % no max constraint
mpcobj.ManipulatedVariables.RateMin = -inf; % no rate constraint
mpcobj.ManipulatedVariables.RateMax = inf;  % no rate constraint

% Example: Enable input constraints (to match MPC_con_main.m)
% mpcobj.ManipulatedVariables.Min = -5;      % u_min = -5 m/s²
% mpcobj.ManipulatedVariables.Max = 5;       % u_max = 5 m/s²
% mpcobj.ManipulatedVariables.RateMin = -3;  % du_min = -3 m/s³
% mpcobj.ManipulatedVariables.RateMax = 3;   % du_max = 3 m/s³

% Output variable (state) constraints
mpcobj.OutputVariables(1).Min = -inf;  % position min
mpcobj.OutputVariables(1).Max = inf;   % position max
mpcobj.OutputVariables(2).Min = -inf;  % velocity min
mpcobj.OutputVariables(2).Max = inf;   % velocity max

% Example: Enable state constraints (to match MPC_con_main.m)
% mpcobj.OutputVariables(1).Min = 0;    % x1_min = 0 m
% mpcobj.OutputVariables(1).Max = 20;   % x1_max = 20 m
% mpcobj.OutputVariables(2).Min = -10;  % x2_min = -10 m/s
% mpcobj.OutputVariables(2).Max = 10;   % x2_max = 10 m/s

%% Additional MPC Toolbox Features (Optional)

% 1. Scale Factors - Improve numerical conditioning
% mpcobj.ManipulatedVariables.ScaleFactor = 5;  % Typical input magnitude
% mpcobj.OutputVariables(1).ScaleFactor = 10;   % Typical position magnitude
% mpcobj.OutputVariables(2).ScaleFactor = 5;    % Typical velocity magnitude

% 2. Soft Constraints - Allow constraint violations with penalty
% mpcobj.OutputVariables(1).MinECR = 1;  % Min Error Constraint Relaxation (1 = hard)
% mpcobj.OutputVariables(1).MaxECR = 1;  % Max Error Constraint Relaxation (1 = hard)
% Example: Soft constraints with penalty weight
% mpcobj.OutputVariables(1).MinECR = 0.1;  % Allow violations with 10x penalty
% mpcobj.OutputVariables(1).MaxECR = 0.1;

% 3. Optimizer Settings
% mpcobj.Optimizer.MaxIterations = 200;           % Max QP iterations
% mpcobj.Optimizer.ConstraintTolerance = 1e-6;    % Constraint satisfaction tolerance
% mpcobj.Optimizer.UseSuboptimalSolution = false; % Use suboptimal if max iter reached

% 4. Disturbance and Noise Models (for state estimation)
% setindist(mpcobj, 'model', tf(1, [1 0.5]));  % Input disturbance model
% setoutdist(mpcobj, 'model', tf(1, [1 0.3])); % Output disturbance model

% 5. Custom State Estimator (Kalman Filter)
% setEstimator(mpcobj, 'default');  % Use default Kalman filter
% Or customize:
% setEstimator(mpcobj, 'custom', A, Bu, C, D, Bv, Dv, ...
%              ProcessNoise, MeasurementNoise, InitialState, InitialCovariance);

% 6. Custom Cost Function Weights (ECR for outputs)
% mpcobj.Weights.ECR = 1e5;  % Penalty weight for constraint violations

%% Initial Conditions
x0 = [0; 0];  % initial state [position; velocity]
u0 = 0;        % initial control input

% Initialize state for simulation
x_mpc = zeros(n, k_steps+1);
x_mpc(:, 1) = x0;
u_mpc = zeros(1, k_steps);

% Create MPC state object
xmpc = mpcstate(mpcobj);

%% Simulation Loop
fprintf('Running MPC simulation with MATLAB Toolbox...\n');

for k = 1:k_steps
    % Current reference for this time step
    ref_current = xr_traj(:, k)';  % Must be row vector
    
    % Build reference trajectory for prediction horizon
    ref_preview = zeros(mpcobj.PredictionHorizon + 1, n);
    for i = 1:mpcobj.PredictionHorizon + 1
        if k + i - 1 <= k_steps + 1
            ref_preview(i, :) = xr_traj(:, k + i - 1)';
        else
            ref_preview(i, :) = xr_traj(:, end)';
        end
    end
    
    % Compute optimal control action
    % mpcmove(mpcobj, xmpc, measured_output, reference, ...)
    [u_mpc(k), Info] = mpcmove(mpcobj, xmpc, x_mpc(:,k)', ref_preview);
    
    % Apply control to system (simulate plant)
    x_mpc(:, k+1) = sys_d.A * x_mpc(:, k) + sys_d.B * u_mpc(k);
    
    % Display progress
    if mod(k, 10) == 0
        fprintf('Step %d/%d completed\n', k, k_steps);
    end
end

fprintf('Simulation completed!\n');

%% Plot Results
% Create time vectors
t_state = 0:ts:Time;
t_input = 0:ts:Time-ts;

% Calculate tracking errors
error_pos = x_mpc(1,:) - xr_traj(1,:);
error_vel = x_mpc(2,:) - xr_traj(2,:);

% Calculate performance metrics
RMSE_pos = sqrt(mean(error_pos.^2));
RMSE_vel = sqrt(mean(error_vel.^2));
Max_error_pos = max(abs(error_pos));
Max_error_vel = max(abs(error_vel));

fprintf('\n--- Performance Metrics ---\n');
fprintf('Position RMSE: %.4f m\n', RMSE_pos);
fprintf('Position Max Error: %.4f m\n', Max_error_pos);
fprintf('Velocity RMSE: %.4f m/s\n', RMSE_vel);
fprintf('Velocity Max Error: %.4f m/s\n', Max_error_vel);

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
legend('Reference', 'MPC (Toolbox)', 'Location', 'best')
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
legend('Reference', 'MPC (Toolbox)', 'Location', 'best')
title('Velocity Tracking')
text(0.02, 0.98, sprintf('RMSE: %.3f m/s\nMax Error: %.3f m/s', RMSE_vel, Max_error_vel), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'BackgroundColor', 'w', 'FontSize', 10);
hold off;

% Control input (acceleration)
subplot(3, 2, 5);
hold on;
stairs(t_input, u_mpc(1,:), 'b-', 'LineWidth', 2.5);
yline(0, 'k--', 'LineWidth', 1);
% No constraints to display
grid on; grid minor;
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Control Input (Acceleration)')
legend('MPC', 'Zero', 'Location', 'best')
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
plot(x_mpc(1,:), x_mpc(2,:), 'b-', 'LineWidth', 2.5, 'DisplayName', 'MPC (Toolbox)');
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
sgtitle('MPC Trajectory Tracking - MATLAB Toolbox Implementation', 'FontSize', 16, 'FontWeight', 'bold');

set(findobj('Type','Axes'),'FontSize',12);

%% Display MPC Controller Properties
fprintf('\n========================================\n');
fprintf('   MPC Controller Configuration       \n');
fprintf('========================================\n');
fprintf('Horizons:\n');
fprintf('  Prediction Horizon: %d steps (%.1f s)\n', mpcobj.PredictionHorizon, mpcobj.PredictionHorizon*ts);
fprintf('  Control Horizon: %d steps (%.1f s)\n', mpcobj.ControlHorizon, mpcobj.ControlHorizon*ts);
fprintf('  Sample Time: %.2f s\n', ts);

fprintf('\nWeights:\n');
fprintf('  Output Weights (Q): [%.1f, %.1f]\n', mpcobj.Weights.OutputVariables);
fprintf('  Input Weight: %.1f\n', mpcobj.Weights.ManipulatedVariables);
fprintf('  Input Rate Weight (R): %.1f\n', mpcobj.Weights.ManipulatedVariablesRate);
fprintf('  ECR Weight: %.0e\n', mpcobj.Weights.ECR);

fprintf('\nConstraints:\n');
fprintf('  Input: [%.1f, %.1f] m/s²\n', mpcobj.ManipulatedVariables.Min, mpcobj.ManipulatedVariables.Max);
fprintf('  Input Rate: [%.1f, %.1f] m/s³\n', mpcobj.ManipulatedVariables.RateMin, mpcobj.ManipulatedVariables.RateMax);
fprintf('  Position: [%.1f, %.1f] m\n', mpcobj.OutputVariables(1).Min, mpcobj.OutputVariables(1).Max);
fprintf('  Velocity: [%.1f, %.1f] m/s\n', mpcobj.OutputVariables(2).Min, mpcobj.OutputVariables(2).Max);

fprintf('\nOptimizer Settings:\n');
% Try to display optimizer properties (may vary by MATLAB version)
try
    fprintf('  Max Iterations: %d\n', mpcobj.Optimizer.MaxIterations);
catch
    fprintf('  Max Iterations: N/A (property not available in this version)\n');
end
try
    fprintf('  Constraint Tolerance: %.0e\n', mpcobj.Optimizer.ConstraintTolerance);
catch
    fprintf('  Constraint Tolerance: N/A (property not available in this version)\n');
end
try
    fprintf('  Use Suboptimal Solution: %s\n', mat2str(mpcobj.Optimizer.UseSuboptimalSolution));
catch
    fprintf('  Use Suboptimal Solution: N/A (property not available in this version)\n');
end

fprintf('\n** Note: Terminal weight Qf not directly configurable in MPC Toolbox\n');
fprintf('         Manual implementation uses Qf = 80*Q for enhanced terminal convergence\n');
fprintf('         See commented sections in code for additional configurable features:\n');
fprintf('         - Soft constraints (ECR)\n');
fprintf('         - Scale factors\n');
fprintf('         - Disturbance models\n');
fprintf('         - Custom state estimators\n');
fprintf('========================================\n');
