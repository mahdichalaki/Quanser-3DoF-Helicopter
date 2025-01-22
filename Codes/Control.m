%% Modern Control - 3-DOF Helicopter System
% University of Tehran - Graduate Course Project
% This script performs modeling, analysis, and control design for a Quanser 3-DOF helicopter.

clear; clc;

%% System Parameters
params = struct(...
    'Mw', 1.87, ...    % Weight mass (kg)
    'Mb', 0.713, ...   % Back rotor mass (kg)
    'Mf', 0.713, ...   % Front rotor mass (kg)
    'g', 9.8, ...      % Gravitational acceleration (m/s^2)
    'La', 0.66, ...    % Distance from pivot to rotor (m)
    'Lw', 0.47, ...    % Distance from pivot to weight (m)
    'Lh', 0.178, ...   % Height to rotor (m)
    'Kf', 0.1188);     % Motor thrust coefficient (N/V)

%% State-Space Representation
A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, ((2*params.La*params.Mf - params.Lw*params.Mw)*params.g) ...
        / (2*params.La^2*params.Mf + 2*params.Lh^2*params.Mf + params.Lw^2*params.Mw), 0, 0, 0, 0];

B = [0, 0;
     0, 0;
     0, 0;
     (params.Kf * params.La) / (2*params.La^2*params.Mf + params.Lw^2*params.Mw), ...
     (params.Kf * params.La) / (2*params.La^2*params.Mf + params.Lw^2*params.Mw);
     0.5 * params.Kf / (params.Lh * params.Mf), -0.5 * params.Kf / (params.Lh * params.Mf);
     0, 0];

C = eye(3, 6);  % Output matrix for position states
D = zeros(3, 2);

%% Stability Analysis

syms s L;
% BIBO Stability
disp('Transfer Function (BIBO Stability):');
G = vpa(C * inv(s * eye(size(A,1)) - A) * B, 3);

% Marginal Stability
disp('Checking Marginal Stability...');
charPoly = charpoly(A);
minPoly = minpoly(A);
eigenvalues = double(solve(det(A - L * eye(size(A,1))) == 0));

% Lyapunov Stability
disp('Checking Lyapunov Stability...');
P = lyap(A, A', eye(size(A,1)));
M = inv(P);
eig_M = eig(M);

%% Observability and Controllability Analysis

disp('Controllability Matrix Rank:');
ctrl_rank = rank(ctrb(A, B));

disp('Observability Matrix Rank:');
obs_rank = rank(obsv(A, C));

%% Pole Placement - Fast Response
disp('Fast Pole Placement...');
Q_fast = diag([500, 10000, 20000, 0, 1000, 0]);
R_fast = diag([0.008, 0.011]);
K_fast = lqr(A, B, Q_fast, R_fast);
x0_fast = [0.03 -0.01 0.02 -0.04 0.03 -0.02]';

%% Pole Placement - Slow Response
disp('Slow Pole Placement...');
Q_slow = diag([10, 10, 4000, 100, 10000, 0]);
R_slow = diag([0.008, 0.011]);
K_slow = lqr(A, B, Q_slow, R_slow);
x0_slow = [0.03 -0.02 0.03 -0.04 0.03 -0.02]';

%% Tracking Control with Integral Action
disp('Tracking Control with Integral Action...');
C1 = [1 0 0 0 0 0; 0 0 1 0 0 0];
A_aug = [A zeros(size(A,1),2); -C1 zeros(2,2)];
B_aug = [B; zeros(2, size(B,2))];
Q_track = diag([40000, 0, 40000, 0, 0, 0, 500000, 600000]);
R_track = diag([0.004, 0.0001]);
K_aug = lqr(A_aug, B_aug, Q_track, R_track);
Ka = K_aug(:, end-1:end);
K = K_aug(:, 1:6);

%% Robustness Analysis with Parameter Variation
disp('Robustness Analysis...');
params.Mw = 0.9 * params.Mw;
params.Mb = 1.1 * params.Mb;
params.Mf = 0.9 * params.Mf;
params.La = 1.1 * params.La;
params.Lw = 1.1 * params.Lw;
params.Lh = 1.1 * params.Lh;

% Update system matrices
A_updated = A;
B_updated = B;

%% Disturbance Rejection
disp('Disturbance Rejection...');
I_value = params.Mw * params.Lw^2 + 2 * params.Mf * (params.La^2 + params.Lh^2);
Disturbance = zeros(6,1);
Disturbance(6) = -params.Lw / I_value;

%% Observer Design
disp('Observer Design...');
P0 = -10 * exp(1i * [15, -15, 30, -30, 40, -40] * pi / 180);
L_observer = place(A', C', P0)';

%% Observer-Based Control
disp('Observer-Based Control...');
F = diag([-10, -20, -30]);
L = diag([-90, -50, -80]);

% First run this section, then open Simulink and run "find_T".
% Execute the following lines after running Simulink:
% T = out.T;
% P = [C; T];

x0_obs = [0 0 0 0.01 0 0.01]';

% Redefine LQR parameters for observer-based control
Q_obs = diag([20000, 10000, 90000, 10000, 20, 100, 500000, 600000]);
R_obs = diag([0.08, 0.04]);
K_obs = lqr(A_aug, B_aug, Q_obs, R_obs);

Ka_obs = K_obs(:, end-1:end);
K_obs = K_obs(:, 1:6);

disp('Completed Modern Control Project Simulation.');