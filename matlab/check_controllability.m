%% design_and_test_state_feedback_poleplace.m
% Same workflow as the previous script but uses pole-placement to compute K.
clear; close all; clc;
fprintf('=== state-feedback (pole-placement) design & nonlinear validation ===\n');

%% 0) Parameters / initialization
init_params; % must create params (17x1) and motor (4x1)

% Unpack needed parameters
b1 = motor(1); b2 = motor(2);
d1 = motor(3); d2 = motor(4);
Bmot = diag([b1 b2]);
Dmot = diag([d1 d2]);

l1  = params(1); l2  = params(2);
a1  = params(3); a2  = params(4);
m1  = params(5); m2  = params(6);
I1  = params(7); I2  = params(8);

%% 1) Define RHS function handle (must match plant_rhs used earlier)
f = @(x,u) plant_rhs(x,u,l1,l2,a1,a2,m1,m2,I1,I2,params(14),params(15),params(16),params(17),Bmot,Dmot);

%% 2) Equilibrium selection
x_eq = [0;0;0;0];
u_eq = [0;0];   % with g=0 and no steady friction this is fine

%% 3) Numerical linearization (finite differences)
nx = 4; nu = 2;
epsFD = 1e-6;

fx0 = f(x_eq,u_eq);
A = zeros(nx);
B = zeros(nx,nu);

for i=1:nx
    dx = zeros(nx,1); dx(i) = epsFD;
    A(:,i) = ( f(x_eq+dx, u_eq) - fx0 ) / epsFD;
end
for j=1:nu
    du = zeros(nu,1); du(j) = epsFD;
    B(:,j) = ( f(x_eq, u_eq+du) - fx0 ) / epsFD;
end

fprintf('\nA matrix:\n'); disp(A);
fprintf('B matrix:\n'); disp(B);

%% 4) Controllability & Observability
Co = ctrb(A,B);
rankCo = rank(Co);
fprintf('Controllability matrix rank = %d (n = %d)\n', rankCo, nx);

C = [1 0 0 0;
     0 1 0 0];
Ob = obsv(A,C);
rankOb = rank(Ob);
fprintf('Observability matrix rank = %d (n = %d)\n', rankOb, nx);

if rankCo < nx
    warning('System not fully controllable at this linearization point. Pole placement may fail or give poor results.');
end

%% 5) Design state-feedback K by pole-placement
% --- Choose desired closed-loop poles (tune these)
% Recommendation: choose moderately fast, well-damped poles.
% Example default: -4, -5, -6, -7  (adjust magnitude to trade speed vs control effort)
desired_poles = [-4; -5; -6; -7];

% If you want faster response: move poles more negative (e.g. -8,-9,...)
% If actuator saturation occurs, choose less aggressive poles (closer to imaginary axis).

% Compute K using place (A_cl = A - B*K will have eigenvalues in desired_poles)
try
    K_place = place(A, B, desired_poles);
catch ME
    error('Pole placement failed: %s\nCheck controllability and choose different poles.', ME.message);
end
K = K_place;  % 2x4

fprintf('\nDesigned K via place (2x4):\n'); disp(K);

%% 6) Closed-loop A_cl and eigenvalues (with sign convention u = -K*(x-xe))
A_cl = A - B*K;
eigA_cl = eig(A_cl);
fprintf('Closed-loop eigenvalues (A_cl = A - B*K):\n'); disp(eigA_cl);

%% 7) Lyapunov certificate: find P s.t. A_cl'*P + P*A_cl = -Q_L
Q_L = eye(nx);
P = lyap(A_cl', Q_L);
eigP = eig(P);
fprintf('Eigenvalues of P (should be > 0):\n'); disp(eigP.');

if all(eigP>0)
    fprintf('Lyapunov certificate found: P positive definite.\n');
else
    warning('P is not positive definite; consider different poles or Q_L.');
end

%% 8) Nonlinear closed-loop simulation
closed_loop_rhs = @(t,x) f(x, u_eq - K*(x - x_eq));  % u = -K*(x-xeq) + u_eq

tspan = [0 5];          % seconds
perturb = [0.1; -0.08; 0.05; -0.03]; % example small perturbation
x0 = x_eq + perturb;

opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t_sim, x_sim] = ode45(@(t,x) closed_loop_rhs(t,x), tspan, x0, opts);

% Compute control history u(t)
u_sim = zeros(length(t_sim),2);
for k=1:length(t_sim)
    xk = x_sim(k,:)';
    u_sim(k,:) = (u_eq - K*(xk - x_eq))';
end

%% 9) Results: plots and checks
figure;
subplot(3,1,1);
plot(t_sim, x_sim(:,1), 'b', 'LineWidth',1.2); hold on;
plot(t_sim, x_sim(:,2), 'r', 'LineWidth',1.2);
ylabel('q (rad)'); legend('q1','q2'); grid on; title('Joint angles (closed-loop)');

subplot(3,1,2);
plot(t_sim, x_sim(:,3), 'b', t_sim, x_sim(:,4), 'r', 'LineWidth',1.2);
ylabel('dq (rad/s)'); legend('dq1','dq2'); grid on;

subplot(3,1,3);
plot(t_sim, u_sim(:,1), 'b', t_sim, u_sim(:,2), 'r','LineWidth',1.2);
ylabel('V (V)'); xlabel('t (s)'); legend('V1','V2'); grid on; title('Control voltages');

dist = vecnorm((x_sim - x_eq')',2)';
figure;
plot(t_sim, dist, 'k','LineWidth',1.2);
xlabel('t (s)'); ylabel('||x - x_{eq}||'); grid on; title('State error norm');

final_err = norm(x_sim(end,:)'-x_eq);
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

% End of script


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Supporting function: state derivative
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xdot = plant_rhs(x,u,l1,l2,a1,a2,m1,m2,I1,I2,m_m1,m_m2,I_m1,I_m2,Bmot,Dmot)

    q = x(1:2);
    dq = x(3:4);

    % Total masses and inertias
    m1t = m1 + m_m1;
    m2t = m2 + m_m2;
    I1t = I1 + I_m1 + I_m2;
    I2t = I2 + I_m2;

    q1 = q(1); q2 = q(2);
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
          h*(dq1^2)];

    % No gravity (planar case)
    G = [0;0];

    % Motor torque
    tau = Bmot*u - Dmot*dq;

    % qdd
    qdd = M \ (tau - Cq - G);

    xdot = [dq; qdd];
end
