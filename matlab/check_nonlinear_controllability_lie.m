function check_nonlinear_controllability_lie()
%% check_nonlinear_controllability_lie.m
% Numerical nonlinear controllability test via Lie brackets over [-pi, pi]^2.
% Uses precomputed link+motor totals from init_params.m. Portable across MATLAB and GNU Octave.

clearvars; close all;

disp('Loading parameters from init_params.m...');
run('init_params.m');

params_local = params;
motor_local  = motor;

%% Define numeric vector fields
f_fun  = @(x) drift_numeric(x, params_local);
g1_fun = @(x) input_numeric(x, params_local, motor_local, 1);
g2_fun = @(x) input_numeric(x, params_local, motor_local, 2);

Lie = @(f_h, g_h, x) ...
    numeric_jacobian(g_h, x)*f_h(x) - ...
    numeric_jacobian(f_h, x)*g_h(x);

%% STEP 1 — CHECK CONTROLLABILITY OVER A GRID (q1, q2), dq1=dq2=0
q1vec = linspace(-pi, pi, 120);
q2vec = linspace(-pi, pi, 120);

[Q1, Q2] = meshgrid(q1vec, q2vec);
rankMap = zeros(size(Q1));
minSigma = inf;

disp('Checking controllability over grid...');

for k = 1:numel(Q1)
    x = [Q1(k); Q2(k); 0; 0];

    g1 = g1_fun(x);
    g2 = g2_fun(x);
    L1 = Lie(f_fun, g1_fun, x);
    L2 = Lie(f_fun, g2_fun, x);

    V = [g1 g2 L1 L2];

    sv = svd(V);
    minSigma = min(minSigma, sv(end));
    rankMap(k) = sum(sv > 1e-6);
end

disp('Controllability check completed.');

%% STEP 2 — EXTRACT ONLY SINGULAR POINTS (rank < 4)
sing_mask = rankMap < 4;
[q1_idx, q2_idx] = find(sing_mask);
sing_points = [q1vec(q1_idx)', q2vec(q2_idx)'];

fprintf('\nGrid points tested: %d\n', numel(Q1));
fprintf('Number of singular points found: %d\n', size(sing_points, 1));
fprintf('Minimum singular value across grid: %.6f\n', minSigma);

if isempty(sing_points)
    disp('The system is controllable (no singular points found).');
else
    disp('WARNING: The system is NOT controllable everywhere - singular points exist.');
end

%% STEP 3 — PLOT SINGULAR POINTS (if GUI display is available)
is_headless = isempty(getenv('DISPLAY')) && exist('OCTAVE_VERSION', 'builtin');
if ~is_headless
    figure;
    if ~isempty(sing_points)
        scatter(sing_points(:, 1), sing_points(:, 2), 8, 'r', 'filled');
    end
    xlabel('q1'); ylabel('q2');
    title('Singular Controllability Points (rank < 4)');
    axis equal; grid on;
end

fprintf('\nDone.\n');
end

%% ==========================================================
% Helper Functions
%% ==========================================================

function fd = drift_numeric(x, params)
    q = x(1:2); dq = x(3:4);
    [M, C, G] = dyn_terms(q, dq, params);
    qdd = M \ (-C - G);
    fd = [dq; qdd];
end

function gi = input_numeric(x, params, motor, i)
    q = x(1:2);
    [M, ~, ~] = dyn_terms(q, [0; 0], params);
    B = diag([motor(1), motor(2)]);
    Gacc = M \ B;
    gi = [0; 0; Gacc(:, i)];
end

function [M, C, G] = dyn_terms(q, dq, params)
    l1 = params(1); l2 = params(2); a1 = params(3); a2 = params(4);
    m1t = params(5); m2t = params(6); I1t = params(7); I2t = params(8);
    g = params(9);

    q1 = q(1); q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);

    c2 = cos(q2); s2 = sin(q2);

    M11 = I1t + I2t + m1t*a1^2 + m2t*(l1^2 + a2^2 + 2*l1*a2*c2);
    M12 = I2t + m2t*(a2^2 + l1*a2*c2);
    M22 = I2t + m2t*a2^2;
    M = [M11 M12; M12 M22];

    h = -m2t*l1*a2*s2;
    C = [h*(2*dq1 + dq2)*dq2;
         -h*(dq1^2)];

    G = [m1t*a1*g*cos(q1) + m2t*g*(l1*cos(q1) + a2*cos(q1+q2));
         m2t*a2*g*cos(q1+q2)];
end

function J = numeric_jacobian(fun, x)
    epsFD = 1e-6;
    f0 = fun(x);
    n = length(x);
    J = zeros(length(f0), n);
    for i = 1:n
        dx = zeros(n, 1); dx(i) = epsFD;
        J(:, i) = (fun(x + dx) - f0) / epsFD;
    end
end
