%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script for evaluating the adaptive control (with time-varying mu) of a 
% single link flexible manipulator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear all; close all;

% Import params, and form mass and stiffness matrices
m_tip = 10;
mu_init = 1;
[L, EI, sig, deg, Ke, M_link, M_tip, M, psi, I_link, I] = get_params(m_tip);

% Build state space matrices
n = size(M, 1);
[A, B, C_hat, D] = build_ABCD(M, Ke, psi, L, n, mu_init);

%% Choose gains and sim parameters (sin wave reference signal)
Kd = 4;
Lambda = 0.2;
Kp = Kd*Lambda;
Gamma = 8;
tspan = [0 16];
ref = @sin_ref_signal;
X0 = [zeros(1,n), 1, zeros(1, n-1)]';

%% Choose gains and sim parameters (monotonically increasing sin wave reference signal)
Kd = 4;
Lambda = 0.2;
Kp = Kd*Lambda;
Gamma = 2000;
Zeta = 0.5;
tspan = [0 40];
ref = @sin_ref_signal_up;
X0 = [0,0,0,0,0,0,0,0]';

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
plot(t, ref(t, L).rho_d, '--', 'LineWidth',2); 
title("Non-adaptive Control with Payload Mass = " + string(m_tip) + ", mu = " + string(mu_init)); grid on;

%% Run ODE solver with adaptive mass control and plot
X0 = [X0; 0];
[t,X] = ode45(@(t,X) adaptiveODE(t, X, n, A, B, C_hat, L, I_link, Kd, Lambda, Gamma, ref), tspan, X0);

% Plot tip position
C = [L, double(subs(psi, L)), zeros(1,n), 0];
rho = C*(X');
subplot(2,1,1);
plot(t, rho, 'LineWidth', 1.5);
hold on;
plot(t, ref(t, L).rho_d, '--'); title("Adaptive Control with Payload Mass = " + string(m_tip) + ", \mu = " + string(mu_init) + ", \Gamma = " + string(Gamma));

% Plot estimated payload mass
subplot(2,1,2);
plot(t, X(:, 9));
title("Estimated payload mass"); grid on;

%% Run ODE solver with adaptive mu control and plot
X0 = [X0; 0; 0];
[t,X] = ode45(@(t,X) adaptiveMuODE(t, X, n, A, B, C_hat, L, I_link, Kd, Lambda, Gamma, Zeta, ref), tspan, X0);
C = [L, double(subs(psi, L)), zeros(1,n), 0, 0];

%% Plot tip position
rho = C*(X');
subplot(3,1,1);
plot(t, rho, 'LineWidth', 1.5);
hold on;
plot(t, ref(t, L).rho_d, '--'); title("Adaptive Control with Payload Mass = " + string(m_tip) + ...
    ", mu = " + string(mu_init) + ", Gamma = " + string(Gamma) + ", Zeta = " + string(Zeta));

% Plot estimated payload mass
subplot(3,1,2);
plot(t, X(:, 9));
title("Estimated payload mass")

% Plot estimated critical mu
subplot(3,1,3);
plot(t, X(:, 10));
title("Estimated critical mu")


%% Plot tip position
rho = C*(X');
plot(t, rho, 'LineWidth', 1.5);
hold on;
plot(t, ref(t, L).rho_d, '--'); title("Adaptive-Mu Control with Payload Mass = " + string(m_tip));

%% Plot a_hat
subplot(2,1,2); plot(t, X(:, 9));
title("Estimated payload mass")




