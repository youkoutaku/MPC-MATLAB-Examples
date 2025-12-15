%% QP_Transform: Transform MPC Cost to Standard QP Form
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   Transforms MPC cost function into standard QP formulation:
%   min 0.5*U'*H*U + x'*F*U
%
% Inputs:
%   A  - Discrete-time system matrix (n×n)
%   B  - Discrete-time input matrix (n×p)
%   Q  - State weighting matrix (n×n)
%   R  - Control rate weighting matrix (p×p)
%   Qf - Terminal state weighting matrix (n×n)
%   Np - Prediction horizon
%
% Outputs:
%   Ap - State prediction matrix [A; A^2; ...; A^Np]
%   Bp - Control prediction matrix (lower triangular)
%   Qp - Extended state weight diag(Q, Q, ..., Qf)
%   Rp - Extended control weight diag(R, R, ..., R)
%   F  - Linear term coefficient (Bp'*Qp*Ap)
%   H  - Quadratic term coefficient (Bp'*Qp*Bp + Rp)
%----------------------------------------------------------%
function [Ap, Bp, Qp, Rp, F, H] = QP_Transform(A, B, Q, R, Qf, Np)
% Input: A, B, Q, R, Qf, Np
% n is the dimension of states
n = size(A,1);
% p is the dimension of inputs
p = size(B,2);
% Define Ap and Bp for QP
Ap = zeros(Np*n, n);
Bp = zeros(Np*n, Np*p);
% Calculate Ap and Bp for QP
for i = 1:Np
    % Ap = [A^1; A^2; ...; A^Np]
    Ap(1+(i-1)*n:i*n,:) = A^i;
    % Bp = [ B          0_{n×p}   0_{n×p}   ...  0_{n×p}
    %        AB         B         0_{n×p}   ...  0_{n×p}
    %        A^2*B       A*B      B         ...  0_{n×p}
    %        ...        ...       ...       ...  ...
    %        A^(h-1)*B  A^(h-2)*B A^(h-3)*B ...  B ]
    for j = 1:i
        Bp((i-1)*n+1:i*n, (j-1)*p+1:j*p) = A^(i-j)*B;
    end
end
% Calculate Qp and Rp for QP
% Qp = diag(Q  Q  ...  Qf)
Qp = kron(eye(Np-1), Q);
Qp = blkdiag(Qp, Qf);
% Rp = diag(R  R  ...  R)
Rp = kron(eye(Np), R);
% F = Bp' * Qp * Ap (for cost formulation)
F = Bp'*Qp*Ap;
% H = Bp' * Qp * Bp + Rp
H = Bp'*Qp*Bp + Rp;
end