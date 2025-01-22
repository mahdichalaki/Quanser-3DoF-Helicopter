%% Modern Control - 3-DOF Helicopter System Modeling
% University of Tehran - Graduate Course Project
% This script models the Quanser 3-DOF helicopter using the Lagrangian method.

clear; clc;
syms lambda(t) epsilon(t) p(t) t Vf Vb Mw Mb Mf g La Lw Lh Kf

%% System Parameters
% Uncomment and modify these values if needed
% Mw = 1.87;      % Weight mass (kg)
% Mb = 0.713;     % Back rotor mass (kg)
% Mf = 0.713;     % Front rotor mass (kg)
% g = 9.8;        % Gravitational acceleration (m/s^2)
% La = 0.66;      % Distance from pivot to rotor (m)
% Lw = 0.47;      % Distance from pivot to weight (m)
% Lh = 0.178;     % Height to rotor (m)
% Kf = 0.1188;    % Motor thrust coefficient (N/V)

%% Coordinate System - Center of Weight (CW)
x_cw(t) = -sin(lambda(t))*cos(epsilon(t))*Lw;
y_cw(t) = -cos(lambda(t))*cos(epsilon(t))*Lw;
z_cw(t) = -sin(epsilon(t))*Lw;

xd_cw = diff(x_cw, t);
yd_cw = diff(y_cw, t);
zd_cw = diff(z_cw, t);

v_cw_2 = simplify(xd_cw^2 + yd_cw^2 + zd_cw^2);

%% Coordinate System - Front Motor (FM)
x_fm = cos(lambda(t)) * cos(p(t)) * Lh - sin(lambda(t)) * sin(epsilon(t)) * sin(p(t)) * Lh + sin(lambda(t)) * cos(epsilon(t)) * La;
y_fm = -sin(lambda(t)) * cos(p(t)) * Lh - cos(lambda(t)) * sin(epsilon(t)) * sin(p(t)) * Lh + cos(lambda(t)) * cos(epsilon(t)) * La;
z_fm = cos(epsilon(t)) * sin(p(t)) * Lh + sin(epsilon(t)) * La;

xd_fm = diff(x_fm, t);
yd_fm = diff(y_fm, t);
zd_fm = diff(z_fm, t);

v_fm_2 = simplify(xd_fm^2 + yd_fm^2 + zd_fm^2);

%% Coordinate System - Back Motor (BM)
x_bm = -cos(lambda(t)) * cos(p(t)) * Lh + sin(lambda(t)) * sin(epsilon(t)) * sin(p(t)) * Lh + sin(lambda(t)) * cos(epsilon(t)) * La;
y_bm = sin(lambda(t)) * cos(p(t)) * Lh + cos(lambda(t)) * sin(epsilon(t)) * sin(p(t)) * Lh + cos(lambda(t)) * cos(epsilon(t)) * La;
z_bm = -cos(epsilon(t)) * sin(p(t)) * Lh + sin(epsilon(t)) * La;

xd_bm = diff(x_bm, t);
yd_bm = diff(y_bm, t);
zd_bm = diff(z_bm, t);

v_bm_2 = simplify(xd_bm^2 + yd_bm^2 + zd_bm^2);

%% Potential Energy Calculation
V_cw = -Mw * g * sin(epsilon(t)) * Lw;
V_fm = Mf * g * (cos(epsilon(t))*sin(p(t)) * Lh + sin(epsilon(t)) * La);
V_bm = Mf * g * (-cos(epsilon(t))*sin(p(t)) * Lh + sin(epsilon(t)) * La);

V_total = simplify(V_cw + V_fm + V_bm);

%% Kinetic Energy Calculation
T_cw = 0.5 * Mw * v_cw_2;
T_fm = 0.5 * Mf * v_fm_2;
T_bm = 0.5 * Mf * v_bm_2;

T_total = simplify(T_cw + T_fm + T_bm);

%% Lagrangian Formulation
L = simplify(T_total - V_total);

%% Generalized Forces
Q1 = La * Kf * (Vf + g * (La*Mf + La*Mf - Lw*Mw) / (La*Kf) + Vb);
Q2 = Kf * (Vf - Vb) * Lh;
Q3 = La * Kf * (Vf + g * (La*Mf + La*Mf - Lw*Mw) / (La*Kf) + Vb) * sin(p(t));

%% Lagrange Equations of Motion
q = [epsilon(t), p(t), lambda(t)];
q_dot = diff(q, t);

equ1 = diff(diff(L, diff(q(1), t)), t) - diff(L, q(1)) - Q1;
equ2 = diff(diff(L, diff(q(2), t)), t) - diff(L, q(2)) - Q2;
equ3 = diff(diff(L, diff(q(3), t)), t) - diff(L, q(3)) - Q3;

%% Nonlinear Equations
equations = [equ1, equ2, equ3];
equations_simplified = simplify(subs(equations, [Kf, Mw, Mf, La, Lh, Lw, g], [0.1188, 1.87, 0.713, 0.660, 0.178, 0.470, 9.81]));

%% Convert to State-Space Form
[V, S] = odeToVectorField(equations_simplified);
V = vpa(simplify(V), 5);

% Generate MATLAB function for nonlinear equations
matlabFunction(V, 'File', 'nonlinear_equations', 'Vars', {'Vf', 'Vb', 'Y'});

%% Output Matrix
C = [eye(3), zeros(3,3)];

%% Summary and Execution
disp('Lagrangian dynamics modeling completed successfully.');
disp('Nonlinear equations are saved as a MATLAB function.');