function check_controllability()
%% check_controllability.m
% Linearizes the 2-DOF manipulator + DC motor model at the origin, checks
% Kalman controllability and observability, computes state-feedback gains via
% pole placement, solves the continuous-time Lyapunov equation, and simulates
% the closed-loop nonlinear plant. Works in both MATLAB and GNU Octave.

clearvars; close all;
fprintf('=== state-feedback (pole-placement) design & nonlinear validation ===\n');

%% 0) Parameters / initialization
init_params; % creates params (17x1) and motor (4x1)

b1 = motor(1); b2 = motor(2);
d1 = motor(3); d2 = motor(4);
Bmot = diag([b1 b2]);
Dmot = diag([d1 d2]);

l1 = params(1); l2 = params(2);
a1 = params(3); a2 = params(4);
m1 = params(5); m2 = params(6);
I1 = params(7); I2 = params(8);

%% 1) Define RHS function handle (using precomputed totals in params(5:8))
f = @(x, u) plant_rhs(x, u, l1, l2, a1, a2, m1, m2, I1, I2, Bmot, Dmot);

%% 2) Equilibrium selection
x_eq = [0; 0; 0; 0];
u_eq = [0; 0];   % planar configuration (g = 0)

%% 3) Numerical linearization (finite differences)
nx = 4; nu = 2;
epsFD = 1e-6;

fx0 = f(x_eq, u_eq);
A = zeros(nx);
B = zeros(nx, nu);

for i = 1:nx
    dx = zeros(nx, 1); dx(i) = epsFD;
    A(:, i) = (f(x_eq + dx, u_eq) - fx0) / epsFD;
end
for j = 1:nu
    du = zeros(nu, 1); du(j) = epsFD;
    B(:, j) = (f(x_eq, u_eq + du) - fx0) / epsFD;
end

fprintf('\nA matrix:\n'); disp(A);
fprintf('B matrix:\n'); disp(B);

%% 4) Controllability & Observability
if exist('ctrb', 'file') || exist('ctrb', 'builtin')
    Co = ctrb(A, B);
else
    Co = [B, A*B, (A^2)*B, (A^3)*B];
end
rankCo = rank(Co);
fprintf('Controllability matrix rank = %d (n = %d)\n', rankCo, nx);

C = [1 0 0 0;
     0 1 0 0];
if exist('obsv', 'file') || exist('obsv', 'builtin')
    Ob = obsv(A, C);
else
    Ob = [C; C*A; C*(A^2); C*(A^3)];
end
rankOb = rank(Ob);
fprintf('Observability matrix rank = %d (n = %d)\n', rankOb, nx);

if rankCo < nx
    warning('System not fully controllable at this linearization point.');
end

%% 5) Design state-feedback K by pole-placement
desired_poles = [-4; -5; -6; -7];

if exist('place', 'file') || exist('place', 'builtin')
    K = place(A, B, desired_poles);
else
    % Analytical decoupled pole placement for M(0)*qdd + Dmot*qd = Bmot*u
    c2 = cos(x_eq(2));
    M11 = I1 + I2 + m1*a1^2 + m2*(l1^2 + a2^2 + 2*l1*a2*c2);
    M12 = I2 + m2*(a2^2 + l1*a2*c2);
    M22 = I2 + m2*a2^2;
    M0 = [M11 M12; M12 M22];
    Kp_des = diag([desired_poles(1)*desired_poles(2), desired_poles(3)*desired_poles(4)]);
    Kd_des = diag([-(desired_poles(1)+desired_poles(2)), -(desired_poles(3)+desired_poles(4))]);
    K = Bmot \ [M0 * Kp_des, M0 * Kd_des - Dmot];
end

fprintf('\nDesigned K (2x4):\n'); disp(K);

%% 6) Closed-loop A_cl and eigenvalues (u = -K*(x - x_eq))
A_cl = A - B*K;
eigA_cl = sort(real(eig(A_cl)));
fprintf('Closed-loop eigenvalues (A_cl = A - B*K):\n'); disp(eigA_cl);

%% 7) Lyapunov certificate: A_cl'*P + P*A_cl = -Q_L
Q_L = eye(nx);
if exist('lyap', 'file') || exist('lyap', 'builtin')
    P = lyap(A_cl', Q_L);
else
    L_op = kron(eye(nx), A_cl') + kron(A_cl', eye(nx));
    p_vec = L_op \ (-Q_L(:));
    P = reshape(p_vec, nx, nx);
    P = 0.5 * (P + P');
end
eigP = eig(P);
fprintf('Eigenvalues of P (should be > 0):\n'); disp(eigP.');

if all(eigP > 0)
    fprintf('Lyapunov certificate found: P positive definite.\n');
else
    warning('P is not positive definite; consider different poles or Q_L.');
end

%% 8) Nonlinear closed-loop simulation
closed_loop_rhs = @(t, x) f(x, u_eq - K*(x - x_eq));

tspan = [0 5];
perturb = [0.1; -0.08; 0.05; -0.03];
x0 = x_eq + perturb;

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_sim, x_sim] = ode45(closed_loop_rhs, tspan, x0, opts);

u_sim = zeros(length(t_sim), 2);
for k = 1:length(t_sim)
    xk = x_sim(k, :)';
    u_sim(k, :) = (u_eq - K*(xk - x_eq))';
end

%% 9) Results: plots (skip GUI windows when headless)
is_headless = isempty(getenv('DISPLAY')) && exist('OCTAVE_VERSION', 'builtin');
if ~is_headless
    figure;
    subplot(3, 1, 1);
    plot(t_sim, x_sim(:, 1), 'b', 'LineWidth', 1.2); hold on;
    plot(t_sim, x_sim(:, 2), 'r', 'LineWidth', 1.2);
    ylabel('q (rad)'); legend('q1', 'q2'); grid on; title('Joint angles (closed-loop)');

    subplot(3, 1, 2);
    plot(t_sim, x_sim(:, 3), 'b', t_sim, x_sim(:, 4), 'r', 'LineWidth', 1.2);
    ylabel('dq (rad/s)'); legend('dq1', 'dq2'); grid on;

    subplot(3, 1, 3);
    plot(t_sim, u_sim(:, 1), 'b', t_sim, u_sim(:, 2), 'r', 'LineWidth', 1.2);
    ylabel('V (V)'); xlabel('t (s)'); legend('V1', 'V2'); grid on; title('Control voltages');

    dist = sqrt(sum((x_sim - x_eq').^2, 2));
    figure;
    plot(t_sim, dist, 'k', 'LineWidth', 1.2);
    xlabel('t (s)'); ylabel('||x - x_{eq}||'); grid on; title('State error norm');
end

final_err = norm(x_sim(end, :)' - x_eq);
fprintf('\nFinal state norm after simulation = %.4e\n', final_err);

%% 10) Print summary
fprintf('\n=== Summary ===\n');
fprintf('Controllability rank: %d (n=4)\n', rankCo);
fprintf('Observability rank:  %d (n=4)\n', rankOb);
fprintf('Closed-loop eigenvalues:\n'); disp(eigA_cl.');
if final_err < 1e-3
    fprintf('Nonlinear closed-loop converged (final error < 1e-3)\n');
else
    fprintf('Nonlinear closed-loop final error = %.4e (may need tuning)\n', final_err);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Supporting function: state derivative
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xdot = plant_rhs(x, u, l1, l2, a1, a2, m1t, m2t, I1t, I2t, Bmot, Dmot)
    q = x(1:2);
    dq = x(3:4);

    q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);

    % Inertia matrix M(q)
    c2 = cos(q2);
    M11 = I1t + I2t + m1t*a1^2 + m2t*(l1^2 + a2^2 + 2*l1*a2*c2);
    M12 = I2t + m2t*(a2^2 + l1*a2*c2);
    M22 = I2t + m2t*a2^2;
    M = [M11 M12; M12 M22];

    % Coriolis term C(q,qdot)*qdot
    h = -m2t*l1*a2*sin(q2);
    Cq = [h*(2*dq1 + dq2)*dq2;
          -h*(dq1^2)];

    % Planar horizontal arm (g = 0)
    G = [0; 0];

    % Motor torque
    tau = Bmot*u - Dmot*dq;

    qdd = M \ (tau - Cq - G);
    xdot = [dq; qdd];
end
