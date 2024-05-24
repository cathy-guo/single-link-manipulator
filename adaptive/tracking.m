clear all;
close all;
clc;

%% Import model params, and define EoMs for beam
L = 1;
EI = 1;
sig = 1;
deg = 3;
m_tip = 10;
mu = 0.8;

% Form psi basis funcs for rayleigh ritz
syms x;
for i = 1:deg
    psi(i) = x^(i+1);
end

% Beam moment of inertia (= 1/3)
I_beam = sig*int(x^2, 0, L);

% Formulate mass matrix for beam
for i = 1:deg
    H(i,1) = sig*int(x*psi(i),0,L);
    for j = 1:deg
        Mee(i,j) = sig*int(psi(i)*psi(j), 0, L);
    end
end
M = double([I_beam H'; H Mee]);

% Formulate stiffness matrix
for i = 1:deg
    for j = 1:deg
        Kee(i,j) = EI * int(diff(psi(i), 2)* diff(psi(j),2), 0, L);
    end
end
Ke = double([0 zeros(1,deg); zeros(deg,1) Kee]);

% Add tip mass effects
I_tip = m_tip * L^2;
for i = 1:deg
    H_tip(i) = m_tip * L*psi(i);
    for j = 1:deg
        Mee_tip(i,j) = m_tip*psi(i)*psi(j);
    end
end
H_tip = subs(H_tip,L);
Mee_tip = subs(Mee_tip, L);
M_tip = double([I_tip, H_tip; H_tip', Mee_tip]);

M = double(M + M_tip);

% Build system matrices
n = size(M, 1);
Bhat = [1, zeros(1,n-1)];

A = [zeros(n), eye(n); -inv(M)*Ke, zeros(n)];
B = double([zeros(n,1); inv(M)*Bhat']);

C_hat = [L, mu*double(subs(psi, L))];
D = 0;

% Controller gains
Kp = 5;
Kd = 10;
K = [-Kp*C_hat, -Kd*C_hat];

% PD Controlled SS representation
Ac = [A + B*K];

double(eig(Ac))

Bc = Kp*B;
Cc = [C_hat, zeros(1,n)];

% Step response
C_hat = [L, mu*double(subs(psi, L))];

% PD Controlled SS representation
Ac = [A + B*K];
Bc = Kp*B;
Cc_mu = [L, mu*double(subs(psi, L)), zeros(1,n)];
Cc_true = [L, double(subs(psi, L)), zeros(1,n)];


sys_step_mu = ss(Ac, Bc, Cc_mu, D);
step(sys_step_mu);
hold on;

sys_step_true = ss(Ac, Bc, Cc_true, D);
step(sys_step_true);
hold on;

title("\mu = " + string(mu))
legend("\mu-tip position", "true tip position")

%%
sqrt(eig(Ke, M))
eig(Ac)

%% Feedforward control tracking a sin wave (True payload mass)
C_hat = [L, mu*double(subs(psi, L))];

Kp = 3.2;
Kd = 4;
tspan = [0 100];
X0 = [zeros(1,n), 1, zeros(1, n-1)]';
I = double(subs(I_beam, L)) %+ I_tip;
[t,X] = ode45(@(t,X) singleLinkODE(t, X, A, B, C_hat, L, I, Kp, Kd), tspan, X0);

C = [L, double(subs(psi, L)), zeros(1,n)];
rho = C*(X');
%%
subplot(2,1,1)
plot(t, rho, 'LineWidth', 1.5)
hold on
plot(t, L*sin(t), '--')
legend("\rho_\mu", "Reference")
ylabel("\rho-tip position")
xlabel("time")
title("Feedforward + PD Control Using an Underestimated Payload Mass")
% title("Tracking Control Performance with Underestimated Payload Mass")
grid on
%%
subplot(2,1,2)
title("Feedforward + PD Control Using True Payload Mass")

%% 
pos = zeros(1, size(X,1));
for xcoord = 0:0.1:L
    Cx = [L, double(subs(psi, L)), zeros(1,n)];
    Cxmu = [L, mu*double(subs(psi, L)), zeros(1,n)];
    pos = [pos; Cx*(X')];
    posmu = [pos; Cxmu*(X')];
end

%%
plot(pos(20,:));
hold on
plot(posmu(20,:), "o");














