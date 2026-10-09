%% Quanser 5-Bar Planar Manipulator: Full State-Feedback Controller & Luenberger Observer
% Authors: Ramez Al-Masadeh & Ahmad Al-Amiri
% Course: Control Systems II (ME0345), German Jordanian University
% Instructor: Dr. Mutaz Ryalat

clear; clc; close all;

%% 1. Physical Parameters (Quanser 5-Bar Manipulator)
L0 = 0.076;      % Distance between base motor axes (m)
L1 = 0.084;      % Length of actuated motor link arms (m)
m1 = 0.01479;    % Mass of link 1 (kg)
m3 = 0.01918;    % Mass of link 2 (kg)
J1 = 6.51e-7;    % Inertia of link 1 (kg*m^2)
J3 = 7.78e-7;    % Inertia of link 2 (kg*m^2)
Jm = 4.00e-6;    % Motor rotor inertia (kg*m^2)

J_total1 = J1 + Jm;  % 4.651e-6 kg*m^2
J_total2 = J3 + Jm;  % 4.778e-6 kg*m^2

%% 2. Target Operating Point & Inverse Kinematics
% Desired end-effector Cartesian coordinates
Ex_des = 0.038;  % 38 mm
Ey_des = 0.080;  % 80 mm

% Numerical Inverse Kinematics via fsolve
% Forward Kinematics relations:
% Ex = (-L1*sin(theta0) + L1*sin(theta1) + L0) / 2
% Ey = ( L1*cos(theta0) + L1*cos(theta1) ) / 2
ik_fun = @(theta) [
    (-L1*sin(theta(1)) + L1*sin(theta(2)) + L0)/2 - Ex_des;
    ( L1*cos(theta(1)) + L1*cos(theta(2)))/2     - Ey_des
];

options = optimoptions('fsolve', 'Display', 'off');
theta_init = [0; 0];
theta_sol = fsolve(ik_fun, theta_init, options);

theta0 = theta_sol(1);
theta1 = theta_sol(2);

fprintf('--- Operating Point Kinematics ---\n');
fprintf('Theta 0: %.4f rad (%.2f deg)\n', theta0, rad2deg(theta0));
fprintf('Theta 1: %.4f rad (%.2f deg)\n', theta1, rad2deg(theta1));

% Forward kinematics verification
Ex_check = (-L1*sin(theta0) + L1*sin(theta1) + L0) / 2;
Ey_check = ( L1*cos(theta0) + L1*cos(theta1)) / 2;
fprintf('Target Position:  Ex = %.4f m, Ey = %.4f m\n', Ex_des, Ey_des);
fprintf('Achieved Position: Ex = %.4f m, Ey = %.4f m\n', Ex_check, Ey_check);

%% 3. Linearized State-Space Formulation
% States: x = [theta0; theta1; dtheta0; dtheta1]
% Inputs: u = [tau0; tau1]
% Outputs: y = [Ex; Ey]

A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

B = [0, 0;
     0, 0;
     1/J_total1, 0;
     0, 1/J_total2];

% Task-space Jacobian output matrix C
dEx_dtheta0 = -0.5 * L1 * cos(theta0);
dEx_dtheta1 =  0.5 * L1 * cos(theta1);
dEy_dtheta0 = -0.5 * L1 * sin(theta0);
dEy_dtheta1 = -0.5 * L1 * sin(theta1);

C = [dEx_dtheta0, dEx_dtheta1, 0, 0;
     dEy_dtheta0, dEy_dtheta1, 0, 0];

D = zeros(2, 2);

%% 4. Controllability & Observability Rank Verification
Co = ctrb(A, B);
Ob = obsv(A, C);

fprintf('\n--- Controllability & Observability ---\n');
fprintf('Controllability Matrix Rank: %d / %d (Full Rank: %d)\n', rank(Co), size(A,1), rank(Co) == size(A,1));
fprintf('Observability Matrix Rank:   %d / %d (Full Rank: %d)\n', rank(Ob), size(A,1), rank(Ob) == size(A,1));

%% 5. Full-State Feedback Pole Placement Design
% Target specs: Settling time < 3 s, Overshoot < 10%
% Desired poles tuned to balance speed, damping, and actuator torque limits:
poles_des = [-2 + 1i, -2 - 1i, -3 + 0.5i, -3 - 0.5i];

K = place(A, B, poles_des);
A_cl = A - B * K;

% Feedforward Precompensator Nbar for Zero Steady-State Error Tracking:
% Nbar scales reference input r so that steady-state output y_ss = r
Nbar = pinv(C * inv(-A_cl) * B);

fprintf('\n--- State-Feedback Gain Matrix K ---\n');
disp(K);
fprintf('--- Precompensator Matrix Nbar ---\n');
disp(Nbar);

%% 6. Luenberger State Observer Design
% Observer poles placed faster than system poles for rapid estimation convergence:
poles_obs = 3 * poles_des; % [-6 +/- 3i, -9 +/- 1.5i]
L = place(A', C', poles_obs)';

fprintf('--- Observer Gain Matrix L ---\n');
disp(L);

%% 7. Augmented Plant-Observer Closed-Loop Simulation
% Augmented state: z = [x; x_hat]
% dx/dt     = A*x - B*K*x_hat + B*Nbar*r
% dx_hat/dt = (A - B*K - L*C)*x_hat + L*C*x + B*Nbar*r
A_aug = [A,           -B*K;
         L*C,  A - B*K - L*C];

B_aug = [B * Nbar;
         B * Nbar];

C_aug = [C, zeros(size(C))];
D_aug = zeros(2, 2);

sys_aug = ss(A_aug, B_aug, C_aug, D_aug);

% Simulation over 5 seconds
t = 0:0.005:5;
r_input = repmat([Ex_des; Ey_des], 1, length(t))';
z0 = zeros(8, 1); % Initial plant and observer states

[y_out, t_out, z_states] = lsim(sys_aug, r_input, t, z0);

%% 8. Plot Step Responses & State Estimation
figure('Name', 'Quanser 5-Bar Control Response', 'NumberTitle', 'off', 'Position', [100, 100, 900, 600]);

subplot(2, 1, 1);
plot(t_out, y_out(:, 1)*1000, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Actual E_x (mm)');
hold on;
yline(Ex_des*1000, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Desired E_x (38 mm)');
grid on;
title('Task-Space Closed-Loop Response with Luenberger Observer: E_x Position');
xlabel('Time (s)'); ylabel('Position (mm)');
legend('Location', 'Southeast');

subplot(2, 1, 2);
plot(t_out, y_out(:, 2)*1000, 'g-', 'LineWidth', 1.8, 'DisplayName', 'Actual E_y (mm)');
hold on;
yline(Ey_des*1000, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Desired E_y (80 mm)');
grid on;
title('Task-Space Closed-Loop Response with Luenberger Observer: E_y Position');
xlabel('Time (s)'); ylabel('Position (mm)');
legend('Location', 'Southeast');
