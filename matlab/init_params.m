%% init_params.m
% Precompute masses & inertias (PAT offsets assumed zero)
% Produces: params (17x1) and motor (4x1)
clear; clc;

%% -------------------------
% 1) Link geometry (edit l1/l2 from CAD if available)
%% -------------------------
l1 = 0.200;   % [m] joint1 -> joint2 (set your CAD value)
l2 = 0.200;   % [m] joint2 -> end effector (set your CAD value)

%% -------------------------
% 2) Link COM positions (from SolidWorks)
%% -------------------------
a1 = 51.62e-3;    % m  (link1 COM from joint1)
a2 = 5.04e-3;     % m  (link2 COM from joint2)

%% -------------------------
% 3) Link masses (SolidWorks -> kg)
%% -------------------------
m1_link = 5.14e-3;   % kg (20.14 g)
m2_link = 5.44e-3;    % kg (8.44 g)

%% -------------------------
% 4) Link inertias about part COM (Izz from SW, g*mm^2 -> kg*m^2)
%    Use the SolidWorks values you collected. These are the values used below.
%% -------------------------
I1_com = 80234.34 * 1e-9;   % kg*m^2  (example from your SW)
I2_com = 14850.27 * 1e-9;   % kg*m^2

%% -------------------------
% 5) Motor CAD values (mass and rotor inertia)
%    (keep these inside params as you requested)
%% -------------------------
m_m1 = 47.45e-3;        % kg (motor 1 mass)
m_m2 = 47.45e-3;        % kg (motor 2 mass)

I_m1_com = 7024.83 * 1e-9;   % kg*m^2 (motor 1 inertia about its COM)
I_m2_com = 7024.83 * 1e-9;   % kg*m^2 (motor 2 inertia about its COM)

%% -------------------------
% 6) ASSUME PAT distances = 0 (you requested this)
%    => motors coaxial: d_m1 = 0 ; d_m2 = 0
%    => no additional m*d^2 terms required
%    (keeps code explicit for future change)
%% -------------------------
d_m1 = 0.0;
d_m2_j2 = 0.0;
% motor2 distance to joint1 (since motor2 is carried by link1) if needed:
d_m2_j1 = l1 + d_m2_j2;  % but d_m2_j2 = 0 so d_m2_j1 = l1

%% -------------------------
% 7) Compute totals (done here so Plant/Controller use ready values)
%    Match exactly the usage in your blocks:
%       m1_total = m1_link + m_m1
%       m2_total = m2_link + m_m2
%       I1_total = I1_com + I_m1_com + I_m2_com   (motor2 reflected to joint1)
%       I2_total = I2_com + I_m2_com
%% -------------------------
m1_total = m1_link + m_m1;
m2_total = m2_link + m_m2;

I1_total = I1_com + I_m1_com + I_m2_com;
I2_total = I2_com + I_m2_com;

%% -------------------------
% 8) Gravity and controller gains
%% -------------------------
g = 0.0;

% Example gains (tune later)
kp1 = 50; kp2 = 50;
kd1 = 10; kd2 = 10;

%% -------------------------
% 9) motor electrical params (separate vector)
%    replace with datasheet values when you have them
%% -------------------------
b1 = 0.8; b2 = 0.8;    % Nm/V (example torque constant)
d1 = 0.01; d2 = 0.01;  % viscous/back-EMF (example)
motor = [b1; b2; d1; d2];

%% -------------------------
% 10) Build final params vector (17x1) IN THE ORDER YOUR BLOCKS EXPECT
%    [l1;l2;a1;a2;m1_total;m2_total;I1_total;I2_total;g;kp1;kp2;kd1;kd2;
%      m_m1;m_m2;I_m1_com;I_m2_com]
%% -------------------------
params = [
    l1;
    l2;
    a1;
    a2;
    m1_total;    % 5
    m2_total;    % 6
    I1_total;    % 7
    I2_total;    % 8
    g;           % 9
    kp1;         %10
    kp2;         %11
    kd1;         %12
    kd2;         %13
    m_m1;        %14
    m_m2;        %15
    I_m1_com;    %16
    I_m2_com     %17
];

%% -------------------------
% 11) Quick verification printout
%% -------------------------
disp('init_params: done.');
disp(['size(params) = ' mat2str(size(params))]);
disp('params = '); disp(params.');
disp('motor = '); disp(motor.');

% small numeric sanity checks
fprintf('m1_total = %.6g kg, m2_total = %.6g kg\n', m1_total, m2_total);
fprintf('I1_total = %.6e kg*m^2, I2_total = %.6e kg*m^2\n', I1_total, I2_total);
