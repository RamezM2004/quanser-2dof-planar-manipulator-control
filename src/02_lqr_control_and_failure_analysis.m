%% Quanser 5-Bar Planar Manipulator: Linear Quadratic Regulator (LQR) & Failure Analysis
% Authors: Ramez Al-Masadeh & Ahmad Al-Amiri
% Course: Control Systems II (ME0345), German Jordanian University

clear; clc; close all;

%% 1. System Matrices (Operating Point)
L0 = 0.076; L1 = 0.084;
J_total1 = 4.651e-6; J_total2 = 4.778e-6;
Ex_des = 0.038; Ey_des = 0.080;

% Compute angles via inverse kinematics
ik_fun = @(theta) [
    (-L1*sin(theta(1)) + L1*sin(theta(2)) + L0)/2 - Ex_des;
    ( L1*cos(theta(1)) + L1*cos(theta(2)))/2     - Ey_des
];
theta_sol = fsolve(ik_fun, [0; 0], optimoptions('fsolve', 'Display', 'off'));
theta0 = theta_sol(1); theta1 = theta_sol(2);

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1/J_total1 0; 0 1/J_total2];
C = [-0.5*L1*cos(theta0), 0.5*L1*cos(theta1), 0, 0;
     -0.5*L1*sin(theta0), -0.5*L1*sin(theta1), 0, 0];
D = zeros(2, 2);

%% 2. LQR Synthesis
% State weighting matrix Q and control effort weighting matrix R
Q = diag([1000, 1000, 10, 10]);
R = diag([0.01, 0.01]);

[K_lqr, S, poles_lqr] = lqr(A, B, Q, R);

fprintf('--- LQR Closed-Loop Poles ---\n');
disp(poles_lqr);

%% 3. Mathematical Analysis of Why Task-Space LQR Failed
% In the experimental report, LQR exhibited severe practical issues:
% 1. Extreme Pole Placement: The resulting closed-loop eigenvalues lie on the order of 10^4 rad/s.
% 2. High-Frequency Dynamics: Demands excessive actuator torque switching (chattering/motor burnout).
% 3. Ill-Conditioned Nbar Precompensator: Inverting (A - B*K_lqr) with high-gain terms leads to
%    extreme numerical sensitivity in pinv(C*inv(-A_cl)*B), causing divergence.

A_cl_lqr = A - B * K_lqr;
Nbar_lqr = pinv(C * inv(-A_cl_lqr) * B);

fprintf('--- LQR Precompensator Matrix Nbar ---\n');
disp(Nbar_lqr);
fprintf('Note the extreme gain magnitudes (order of 10^3 to 10^4), indicating numerical instability.\n');
