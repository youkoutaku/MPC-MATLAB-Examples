%% MPC Utility Functions
% Collection of helper functions for MPC implementations
%
% Author: Guang-Ze Yang
% Date: 2025

%% Function: Build Lifted Matrices
function [Sx, Su] = build_lifted_matrices(A, B, N)
    % Build lifted system matrices for MPC formulation
    %
    % Inputs:
    %   A - State transition matrix (nx x nx)
    %   B - Input matrix (nx x nu)
    %   N - Prediction horizon
    %
    % Outputs:
    %   Sx - State prediction from initial state (nx*N x nx)
    %   Su - State prediction from input sequence (nx*N x nu*N)
    %
    % The predicted states are: X = Sx*x0 + Su*U
    % where X = [x(1); x(2); ...; x(N)]
    %       U = [u(0); u(1); ...; u(N-1)]
    
    nx = size(A, 1);
    nu = size(B, 2);
    
    Sx = zeros(nx*N, nx);
    Su = zeros(nx*N, nu*N);
    
    for i = 1:N
        % Sx block: x(i) = A^i * x0 + ...
        Sx((i-1)*nx+1:i*nx, :) = A^i;
        
        % Su block: contribution from each input
        for j = 1:i
            Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j)*B;
        end
    end
end

%% Function: Build MPC Cost Matrices
function [H, f] = build_mpc_cost(Sx, Su, x0, X_ref, Q_bar, R_bar)
    % Build quadratic cost function matrices for MPC
    %
    % Cost: 0.5 * U'*H*U + f'*U
    %
    % Inputs:
    %   Sx     - State prediction from x0
    %   Su     - State prediction from U
    %   x0     - Initial state
    %   X_ref  - Reference state trajectory (stacked vector)
    %   Q_bar  - Block diagonal state weight matrix
    %   R_bar  - Block diagonal input weight matrix
    %
    % Outputs:
    %   H - Hessian matrix
    %   f - Linear term vector
    
    H = Su'*Q_bar*Su + R_bar;
    f = Su'*Q_bar*(Sx*x0 - X_ref);
    
    % Symmetrize for numerical stability
    H = (H + H')/2;
end

%% Function: Compute Terminal Cost using DARE
function P = compute_terminal_cost(A, B, Q, R)
    % Compute terminal cost matrix as solution to Discrete Algebraic Riccati Equation
    %
    % Inputs:
    %   A - State transition matrix
    %   B - Input matrix
    %   Q - State weight matrix
    %   R - Input weight matrix
    %
    % Output:
    %   P - Terminal cost matrix (solution to DARE)
    
    try
        P = dare(A, B, Q, R);
    catch
        warning('DARE solution failed, using Q as terminal cost');
        P = Q;
    end
end

%% Function: Check MPC Stability Conditions
function is_stable = check_mpc_stability(A, B, Q, R)
    % Check basic stability conditions for MPC
    %
    % Checks:
    %   1. System is stabilizable
    %   2. System is detectable
    %   3. Q is positive semidefinite
    %   4. R is positive definite
    %
    % Inputs:
    %   A, B - System matrices
    %   Q, R - Weight matrices
    %
    % Output:
    %   is_stable - Boolean indicating if conditions are satisfied
    
    is_stable = true;
    
    % Check if system is stabilizable
    if rank(ctrb(A, B)) < size(A, 1)
        warning('System is not controllable/stabilizable');
        is_stable = false;
    end
    
    % Check Q is positive semidefinite
    eigQ = eig(Q);
    if any(eigQ < -1e-10)
        warning('Q is not positive semidefinite');
        is_stable = false;
    end
    
    % Check R is positive definite
    eigR = eig(R);
    if any(eigR <= 1e-10)
        warning('R is not positive definite');
        is_stable = false;
    end
end

%% Function: Discretize Continuous System
function [Ad, Bd] = discretize_system(Ac, Bc, Ts, method)
    % Discretize continuous-time system
    %
    % Inputs:
    %   Ac, Bc - Continuous-time system matrices
    %   Ts     - Sampling time
    %   method - Discretization method ('zoh', 'tustin', 'euler')
    %            Default: 'zoh' (Zero-Order Hold)
    %
    % Outputs:
    %   Ad, Bd - Discrete-time system matrices
    
    if nargin < 4
        method = 'zoh';
    end
    
    switch lower(method)
        case 'zoh'
            % Zero-order hold (exact discretization)
            sys_c = ss(Ac, Bc, eye(size(Ac)), 0);
            sys_d = c2d(sys_c, Ts, 'zoh');
            Ad = sys_d.A;
            Bd = sys_d.B;
            
        case 'euler'
            % Forward Euler method
            Ad = eye(size(Ac)) + Ts*Ac;
            Bd = Ts*Bc;
            
        case 'tustin'
            % Tustin (bilinear) method
            sys_c = ss(Ac, Bc, eye(size(Ac)), 0);
            sys_d = c2d(sys_c, Ts, 'tustin');
            Ad = sys_d.A;
            Bd = sys_d.B;
            
        otherwise
            error('Unknown discretization method: %s', method);
    end
end

%% Function: Compute MPC Performance Metrics
function metrics = compute_mpc_metrics(x_history, u_history, x_ref, u_ref)
    % Compute performance metrics for MPC
    %
    % Inputs:
    %   x_history - State trajectory (nx x T)
    %   u_history - Input trajectory (nu x T)
    %   x_ref     - Reference state (nx x 1) or (nx x T)
    %   u_ref     - Reference input (nu x 1) or (nu x T), optional
    %
    % Output:
    %   metrics - Structure containing performance metrics
    
    [nx, T] = size(x_history);
    [nu, ~] = size(u_history);
    
    % Expand reference if constant
    if size(x_ref, 2) == 1
        x_ref = repmat(x_ref, 1, T);
    end
    
    % State tracking error
    x_error = x_history - x_ref;
    metrics.state_rmse = sqrt(mean(sum(x_error.^2, 1)));
    metrics.state_max_error = max(sqrt(sum(x_error.^2, 1)));
    
    % Control effort
    metrics.control_rms = sqrt(mean(sum(u_history.^2, 1)));
    metrics.control_max = max(abs(u_history(:)));
    
    % Control variation (smoothness)
    du = diff(u_history, 1, 2);
    metrics.control_variation = sqrt(mean(sum(du.^2, 1)));
    
    % Settling time (state error below 2% of initial error)
    initial_error = sqrt(sum(x_error(:,1).^2));
    threshold = 0.02 * initial_error;
    error_norm = sqrt(sum(x_error.^2, 1));
    settling_idx = find(error_norm <= threshold, 1, 'first');
    if ~isempty(settling_idx)
        metrics.settling_time = settling_idx;
    else
        metrics.settling_time = T;
    end
    
    % If reference input provided, compute input tracking error
    if nargin > 3 && ~isempty(u_ref)
        if size(u_ref, 2) == 1
            u_ref = repmat(u_ref, 1, T);
        end
        u_error = u_history - u_ref;
        metrics.input_rmse = sqrt(mean(sum(u_error.^2, 1)));
    end
end

%% Function: Plot MPC Results
function plot_mpc_results(t, x_history, u_history, x_ref, u_min, u_max)
    % Create standard plots for MPC results
    %
    % Inputs:
    %   t         - Time vector
    %   x_history - State trajectory (nx x T)
    %   u_history - Input trajectory (nu x T)
    %   x_ref     - Reference state (nx x 1) or (nx x T)
    %   u_min     - Input lower bound (optional)
    %   u_max     - Input upper bound (optional)
    
    [nx, T] = size(x_history);
    [nu, ~] = size(u_history);
    
    % Expand reference if constant
    if size(x_ref, 2) == 1
        x_ref = repmat(x_ref, 1, T);
    end
    
    figure('Position', [100, 100, 1000, 600]);
    
    % Plot states
    for i = 1:nx
        subplot(nx+1, 1, i);
        plot(t, x_history(i,:), 'b-', 'LineWidth', 2);
        hold on;
        plot(t, x_ref(i,:), 'r--', 'LineWidth', 1.5);
        grid on;
        xlabel('Time');
        ylabel(sprintf('State x_%d', i));
        if i == 1
            legend('Actual', 'Reference', 'Location', 'best');
        end
    end
    
    % Plot control input
    subplot(nx+1, 1, nx+1);
    stairs(t, u_history, 'r-', 'LineWidth', 2);
    hold on;
    if nargin > 4 && ~isempty(u_min)
        plot(t, u_min*ones(1,T), 'k--', 'LineWidth', 1);
    end
    if nargin > 5 && ~isempty(u_max)
        plot(t, u_max*ones(1,T), 'k--', 'LineWidth', 1);
    end
    grid on;
    xlabel('Time');
    ylabel('Control Input u');
    legend('Control Input', 'Location', 'best');
end
