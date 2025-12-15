%% Augmented_System_Input: Create Augmented System (Legacy)
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   Creates augmented system including input increments.
%   This is a legacy function kept for reference.
%
% Status: DEPRECATED - Not used in current implementation
%
% Inputs:
%   A  - State matrix (n×n)
%   B  - Input matrix (n×p)
%   Q  - State cost matrix (n×n)
%   R  - Input cost matrix (p×p)
%   S  - Terminal cost matrix (n×n)
%   Ar - Reference matrix (n×n)
%
% Outputs:
%   Az - Augmented state matrix
%   Bz - Augmented input matrix
%   Qz - Augmented state cost
%   Rz - Augmented input cost
%   Sz - Augmented terminal cost
%----------------------------------------------------------%
function [Az,Bz,Qz,Rz,Sz] = Augmented_System_Input(A,B,Q,R,S,Ar)
% A: State matrix
% B: Input matrix
% Q: State cost matrix
% R: Input cost matrix
% S: Terminal cost matrix
% Ar: Reference matrix
n=size(A,1);
p=size(B,2);
% Ca = [I -I 0]
Ca =[eye(n) -eye(n) zeros(n,p)];
% Az = [A 0 B;0 Ar 0;0 0 I]
Az = [A zeros(n) B;zeros(n) Ar zeros(n,p);zeros(p,n) zeros(p,n) eye(p,p)];
% Bz = [B; 0; I]
Bz = [B;zeros(n,p);eye(p)];
% Qz = Ca^T * Q * Ca
Qz = Ca.'*Q*Ca;
% Sz = Ca^T * S * Ca
Sz = Ca.'*S*Ca;
% Rz = R
Rz = R;
end