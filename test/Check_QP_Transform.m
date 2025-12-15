%% Unit Test for QP_Transform Function
%----------------------------------------------------------%
% Project: Model Predictive Control Implementation
% Author:  Youkoutaku (https://youkoutaku.github.io/)
% License: MIT License
% Repository: https://github.com/Youkoutaku/My_Optimal_Control
%----------------------------------------------------------%
% Description:
%   Unit test to verify QP_Transform function correctness.
%   Tests prediction matrix construction and QP formulation.
%
% Features:
%   - Tests Ap (state prediction) matrix structure
%   - Tests Bp (control prediction) matrix structure
%   - Verifies Qp, Rp, F, H dimensions
%
% Usage:
%   cd test
%   addpath('../lib')
%   run('Check_QP_Transform.m')
%----------------------------------------------------------%
clear;
close all;
clc;
addpath('../lib');  % Add library folder

A = [1 0; 0 1];
B = [0 0.2;-0.1 0.5];
n = size (A,1);
p = size (B,2);

Q = eye(n);
Qf= eye(n);
R = 0.1*eye(n);

h = 2;

[Ap, Bp, Qp, Rp, F, H] = QP_Transform(A, B, Q, R, Qf,2);

disp(Ap)