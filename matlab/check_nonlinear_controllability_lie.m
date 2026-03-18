%% check_controllability_and_singularities.m
% Numerical nonlinear controllability test + singularity plot
% Uses unchanged init_params.m

clear; close all; clc;

disp("Loading parameters from init_params.m...");
run('init_params.m');   % your file, unchanged

%% Extract robot parameters for numeric functions
params_local = params;
motor_local  = motor;

%% Define numeric vector fields
f_fun  = @(x) drift_numeric(x, params_local, motor_local);
g1_fun = @(x) input_numeric(x, params_local, motor_local, 1);
g2_fun = @(x) input_numeric(x, params_local, motor_local, 2);

% Lie bracket operator (numeric Jacobian)
Lie = @(f_fun, g_fun, x) ...
    numeric_jacobian(g_fun, x)*f_fun(x) - ...
    numeric_jacobian(f_fun, x)*g_fun(x);

%% --------------------------------------------------------
% STEP 1 — CHECK CONTROLLABILITY OVER A GRID (q1,q2), dq1=dq2=0
%% --------------------------------------------------------

q1vec = linspace(-pi, pi, 120);
q2vec = linspace(-pi, pi, 120);

[Q1, Q2] = meshgrid(q1vec, q2vec);
rankMap = zeros(size(Q1));

disp("Checking controllability over grid...");

for k = 1:numel(Q1)
    x = [Q1(k); Q2(k); 0; 0];

    g1 = g1_fun(x);
    g2 = g2_fun(x);
    L1 = Lie(f_fun, g1_fun, x);
    L2 = Lie(f_fun, g2_fun, x);

    V = [g1 g2 L1 L2];

    r = rank(V, 1e-6);
    rankMap(k) = r;
end

disp("Controllability check completed.");

%% --------------------------------------------------------
% STEP 2 — EXTRACT ONLY SINGULAR POINTS (rank < 4)
%% --------------------------------------------------------

sing_mask = rankMap < 4;

[q1_idx, q2_idx] = find(sing_mask);

sing_points = [
    q1vec(q1_idx)' , q2vec(q2_idx)'   % list of (q1,q2)
];

fprintf("\nNumber of singular points found: %d\n", size(sing_points,1));

if isempty(sing_points)
    disp("✔ The system is controllable (no singular points found).");
else
    disp("⚠ The system is NOT controllable everywhere — singular points exist.");
end


%% --------------------------------------------------------
% STEP 3 — PLOT ONLY THE SINGULAR POINTS
%% --------------------------------------------------------

figure;
scatter(sing_points(:,1), sing_points(:,2), 8, 'r', 'filled');
xlabel('q1'); ylabel('q2');
title('Singular Controllability Points (rank < 4)');
axis equal; grid on;

disp("Plotting complete — singular points shown.");
fprintf("\nDone.\n");


%% ==========================================================
% Helper Functions
%% ==========================================================

function fd = drift_numeric(x, params, motor)
    q=x(1:2); dq=x(3:4);
    [M,C,G] = dyn_terms(q,dq,params);
    qdd = M \ (-C - G);
    fd = [dq; qdd];
end

function gi = input_numeric(x, params, motor, i)
    q = x(1:2);
    [M,~,~] = dyn_terms(q,[0;0],params);
    B = diag([motor(1), motor(2)]);
    Gacc = M \ B;
    gi = [0;0; Gacc(:,i)];
end

function [M,C,G] = dyn_terms(q,dq,params)
    l1=params(1); l2=params(2); a1=params(3); a2=params(4);
    m1=params(5); m2=params(6); I1=params(7); I2=params(8); g=params(9);
    m_m1=params(14); m_m2=params(15); I_m1=params(16); I_m2=params(17);

    m1t=m1+m_m1; m2t=m2+m_m2;
    I1t=I1+I_m1+I_m2; I2t=I2+I_m2;

    q1=q(1); q2=q(2);
    dq1=dq(1); dq2=dq(2);

    c2 = cos(q2); s2 = sin(q2);

    % Inertia matrix
    M11 = I1t+I2t + m1t*a1^2 + m2t*(l1^2 + a2^2 + 2*l1*a2*c2);
    M12 = I2t + m2t*(a2^2 + l1*a2*c2);
    M22 = I2t + m2t*a2^2;
    M = [M11 M12; M12 M22];

    % Coriolis
    h = -m2t*l1*a2*s2;
    C = [h*(2*dq1 + dq2)*dq2;
         h*(dq1^2)];

    % Gravity
    G = [m1t*a1*g*cos(q1) + m2t*g*(l1*cos(q1) + a2*cos(q1+q2));
         m2t*a2*g*cos(q1+q2)];
end

function J = numeric_jacobian(fun, x)
    eps = 1e-6;
    f0 = fun(x);
    n = length(x);
    J = zeros(length(f0), n);
    for i = 1:n
        dx = zeros(n,1); dx(i)=eps;
        J(:,i) = (fun(x+dx) - f0) / eps;
    end
end
