%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script for evaluating the adaptive control (with constant mu) of a single link flexible
% manipulator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear all; close all;

%% Import params, and form mass and stiffness matrices
m_tip = 10;
mu = 0.95;
[L, EI, sig, deg, Ke, M_link, M_tip, M, psi, I_link, I] = get_params(m_tip);

% Build state space matrices
n = size(M, 1);
[A, B, C_hat, D] = build_ABCD(M, Ke, psi, L, n, mu);

%% Choose gains and sim parameters (sin wave reference signal)
Kd = 4;
Lambda = 0.2;
Kp = Kd*Lambda;
Gamma = 8;
tspan = [0 60];
ref = @sin_ref_signal;
X0 = [zeros(1,n), 1, zeros(1, n-1)]';

%% Choose spline reference signal
Kd = 1;
Lambda = 0.5;
Kp = Kd*Lambda;
Gamma = 8;
tspan = [0 20];
ref = @spline_ref_signal;
thetadot0 = spline_ref_signal(0, L).theta_d_dot;
X0 = [zeros(1,n), 1, zeros(1, n-1)]';

%% Run ODE solver with simple controller and plot

[t,X] = ode45(@(t,X) singleLinkODE(t, X, A, B, C_hat, L, I, Kp, Kd, ref), tspan, X0);
U = zeros(length(t), 1);
for i = 1:length(t)
    [~, u] = singleLinkODE(t(i), X(i,:)', A, B, C_hat, L, I, Kp, Kd, ref);
    U(i, 1) = u;
end
% Plot tip position
C = [L, double(subs(psi, L)), zeros(1,n)];
rho = C*(X');
plot(t, rho, 'LineWidth', 1.5); hold on;
% plot(t, ref(t, L).theta_d_dd, 'g');
plot(t, ref(t, L).rho_d, '--'); title("payload mass = ", string(m_tip));
title("Non-adaptive Control with Payload Mass = " + string(m_tip));

%% Run ODE solver with adaptive control and plot
X0 = [zeros(1,n), 1, zeros(1, n)]';
[t,X] = ode45(@(t,X) adaptiveODE(t, X, n, A, B, C_hat, L, I_link, Kd, Lambda, Gamma, ref), tspan, X0);

% Plot tip position
C = [L, double(subs(psi, L)), zeros(1,n), 0];
rho = C*(X');
subplot(2,1,1); plot(t, rho, 'LineWidth', 1.5);
hold on;
plot(t, ref(t, L).rho_d, '--'); title("Adaptive Control with Payload Mass = " + string(m_tip) + ", \mu = " + string(mu));

% Plot estimated payload mass
subplot(2,1,2); plot(t, X(:, 9));
title("Estimated payload mass")




