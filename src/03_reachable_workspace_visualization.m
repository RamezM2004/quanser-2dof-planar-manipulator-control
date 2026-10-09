%% Quanser 5-Bar Planar Manipulator: Reachable Workspace Visualization
% Authors: Ramez Al-Masadeh & Ahmad Al-Amiri
% Course: Control Systems II (ME0345), German Jordanian University

clear; clc; close all;

L0 = 0.076; % Distance between motors (m)
L1 = 0.084; % Link length (m)
Ex_des = 0.038;
Ey_des = 0.080;

figure('Name', 'Reachable Workspace Analysis', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 450]);

%% 1. Theoretical Unconstrained Workspace (-pi to pi)
theta_full = linspace(-pi, pi, 150);
[T0_full, T1_full] = meshgrid(theta_full, theta_full);

Ex_full = (-L1*sin(T0_full) + L1*sin(T1_full) + L0) / 2;
Ey_full = ( L1*cos(T0_full) + L1*cos(T1_full)) / 2;

subplot(1, 2, 1);
plot(Ex_full(:)*1000, Ey_full(:)*1000, '.', 'Color', [0.8 0.8 0.9]);
hold on;
plot(Ex_des*1000, Ey_des*1000, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
grid on; axis equal;
title('Theoretical Workspace: \theta \in [-\pi, \pi]');
xlabel('E_x (mm)'); ylabel('E_y (mm)');
legend('Reachable Points', 'Operating Point (38, 80 mm)', 'Location', 'South');

%% 2. Physically Constrained Workspace (-pi/2 to pi/2)
theta_lim = linspace(-pi/2, pi/2, 150);
[T0_lim, T1_lim] = meshgrid(theta_lim, theta_lim);

Ex_lim = (-L1*sin(T0_lim) + L1*sin(T1_lim) + L0) / 2;
Ey_lim = ( L1*cos(T0_lim) + L1*cos(T1_lim)) / 2;

subplot(1, 2, 2);
plot(Ex_lim(:)*1000, Ey_lim(:)*1000, '.', 'Color', [0.2 0.6 0.3]);
hold on;
plot(Ex_des*1000, Ey_des*1000, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
grid on; axis equal;
title('Physical Constrained Workspace: \theta \in [-\pi/2, \pi/2]');
xlabel('E_x (mm)'); ylabel('E_y (mm)');
legend('Reachable Points', 'Operating Point (38, 80 mm)', 'Location', 'South');
