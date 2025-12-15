%% Constraint Verification for MPC_con_main
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   Verification tool to check constraint satisfaction in MPC_con_main.
%   Detects and reports any violations of input, state, or rate constraints.
%
% Features:
%   - Detailed violation detection (tolerance: 1e-6)
%   - Constraint activation statistics
%   - Verification plots for all constraint types
%
% Usage:
%   cd test
%   run('Check_Constraints.m')
%   % or from MPC folder:
%   run('test/Check_Constraints.m')
%----------------------------------------------------------%

% Run the main MPC simulation
cd ..
addpath('lib')
run('MPC_con_main.m')
cd test

%% Verify Constraints
fprintf('\n========================================\n');
fprintf('   Constraint Verification Results     \n');
fprintf('========================================\n');

% 1. Check Input Constraints
u_violations_detailed = zeros(k_steps, 1);
for k = 1:k_steps
    if u_mpc(1,k) > u_max + 1e-6
        u_violations_detailed(k) = u_mpc(1,k) - u_max;
        fprintf('Input UPPER violation at step %d: u = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, u_mpc(1,k), u_max, u_violations_detailed(k));
    elseif u_mpc(1,k) < u_min - 1e-6
        u_violations_detailed(k) = u_min - u_mpc(1,k);
        fprintf('Input LOWER violation at step %d: u = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, u_mpc(1,k), u_min, -u_violations_detailed(k));
    end
end

% 2. Check Velocity Constraints
vel_violations_detailed = zeros(k_steps+1, 1);
for k = 1:k_steps+1
    if x_mpc(2,k) > x2_max + 1e-6
        vel_violations_detailed(k) = x_mpc(2,k) - x2_max;
        fprintf('Velocity UPPER violation at step %d: v = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, x_mpc(2,k), x2_max, vel_violations_detailed(k));
    elseif x_mpc(2,k) < x2_min - 1e-6
        vel_violations_detailed(k) = x2_min - x_mpc(2,k);
        fprintf('Velocity LOWER violation at step %d: v = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, x_mpc(2,k), x2_min, -vel_violations_detailed(k));
    end
end

% 3. Check Position Constraints
pos_violations_detailed = zeros(k_steps+1, 1);
for k = 1:k_steps+1
    if x_mpc(1,k) > x1_max + 1e-6
        pos_violations_detailed(k) = x_mpc(1,k) - x1_max;
        fprintf('Position UPPER violation at step %d: p = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, x_mpc(1,k), x1_max, pos_violations_detailed(k));
    elseif x_mpc(1,k) < x1_min - 1e-6
        pos_violations_detailed(k) = x1_min - x_mpc(1,k);
        fprintf('Position LOWER violation at step %d: p = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, x_mpc(1,k), x1_min, -pos_violations_detailed(k));
    end
end

% 4. Check Control Rate Constraints
du_actual = [u_mpc(1,1), diff(u_mpc(1,:))];
du_violations_detailed = zeros(k_steps, 1);
for k = 1:k_steps
    if du_actual(k) > du_max + 1e-6
        du_violations_detailed(k) = du_actual(k) - du_max;
        fprintf('Rate UPPER violation at step %d: du = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, du_actual(k), du_max, du_violations_detailed(k));
    elseif du_actual(k) < du_min - 1e-6
        du_violations_detailed(k) = du_min - du_actual(k);
        fprintf('Rate LOWER violation at step %d: du = %.4f (limit: %.4f, violation: %.4f)\n', ...
            k, du_actual(k), du_min, -du_violations_detailed(k));
    end
end

%% Summary Statistics
fprintf('\n========================================\n');
fprintf('Summary:\n');
fprintf('  Total input violations: %d (max: %.4f)\n', sum(u_violations_detailed > 0), max(abs(u_violations_detailed)));
fprintf('  Total velocity violations: %d (max: %.4f)\n', sum(vel_violations_detailed > 0), max(abs(vel_violations_detailed)));
fprintf('  Total position violations: %d (max: %.4f)\n', sum(pos_violations_detailed > 0), max(abs(pos_violations_detailed)));
fprintf('  Total rate violations: %d (max: %.4f)\n', sum(du_violations_detailed > 0), max(abs(du_violations_detailed)));

% Check input constraint bounds
fprintf('\n========================================\n');
fprintf('Input Constraint Statistics:\n');
fprintf('  u_min = %.2f, u_max = %.2f\n', u_min, u_max);
fprintf('  Actual u range: [%.4f, %.4f]\n', min(u_mpc), max(u_mpc));
fprintf('  Input hits upper limit: %d times\n', sum(abs(u_mpc - u_max) < 1e-4));
fprintf('  Input hits lower limit: %d times\n', sum(abs(u_mpc - u_min) < 1e-4));

fprintf('\n========================================\n');
fprintf('Velocity Constraint Statistics:\n');
fprintf('  x2_min = %.2f, x2_max = %.2f\n', x2_min, x2_max);
fprintf('  Actual velocity range: [%.4f, %.4f]\n', min(x_mpc(2,:)), max(x_mpc(2,:)));
fprintf('  Velocity hits upper limit: %d times\n', sum(abs(x_mpc(2,:) - x2_max) < 1e-4));
fprintf('  Velocity hits lower limit: %d times\n', sum(abs(x_mpc(2,:) - x2_min) < 1e-4));

%% Visualization
figure('Position', [100, 100, 1200, 400], 'Color', 'w', 'Name', 'Constraint Verification')

% Input with limits
subplot(1,3,1)
stairs(t_input, u_mpc(1,:), 'b-', 'LineWidth', 2); hold on;
yline(u_max, 'r--', 'LineWidth', 2);
yline(u_min, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Acceleration (m/s²)', 'FontWeight', 'bold')
title('Input: Checking u_{min} ≤ u ≤ u_{max}', 'FontSize', 11, 'FontWeight', 'bold')
legend('Control Input', 'Limits', 'Location', 'best')
xlim([0 Time])

% Velocity with limits
subplot(1,3,2)
plot(t_state, x_mpc(2,:), 'b-', 'LineWidth', 2); hold on;
yline(x2_max, 'r--', 'LineWidth', 2);
yline(x2_min, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Velocity (m/s)', 'FontWeight', 'bold')
title('Velocity: Checking x₂_{min} ≤ x₂ ≤ x₂_{max}', 'FontSize', 11, 'FontWeight', 'bold')
legend('Velocity', 'Limits', 'Location', 'best')
xlim([0 Time])

% Control rate with limits
subplot(1,3,3)
stairs(t_input, du_actual, 'b-', 'LineWidth', 2); hold on;
yline(du_max, 'r--', 'LineWidth', 2);
yline(du_min, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)', 'FontWeight', 'bold')
ylabel('Jerk (m/s³)', 'FontWeight', 'bold')
title('Rate: Checking Δu_{min} ≤ Δu ≤ Δu_{max}', 'FontSize', 11, 'FontWeight', 'bold')
legend('Control Rate', 'Limits', 'Location', 'best')
xlim([0 Time])

sgtitle('\bf Constraint Verification: All Constraints Should Be Satisfied', 'FontSize', 12)
set(findobj('Type','Axes'),'FontSize',10, 'LineWidth', 1.2);
