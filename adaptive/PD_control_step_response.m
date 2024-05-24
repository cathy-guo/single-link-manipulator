clear all;
close all;
clc;

%% Import model params, and define EoMs for beam
L = 1;
EI = 1;
sig = 9;
deg = 3;
m_tip = 1;

% Form psi basis funcs for rayleigh ritz
syms x;
for i = 1:deg
    psi(i) = x^(i+1);
end

% Beam moment of inertia
I = sig*int(x^2, 0, L);

% Formulate mass matrix
for i = 1:deg
    H(i,1) = sig*int(x*psi(i),0,L);
    for j = 1:deg
        Mee(i,j) = sig*int(psi(i)*psi(j),0,L);
    end
end
M = double([I H'; H Mee]);

% Formulate stiffness matrix
for i = 1:deg
    for j = 1:deg
        Kee(i,j) = EI * int(diff(psi(i), 2)* diff(psi(j),2), 0,1);
    end
end
Ke = double([0 zeros(1,deg); zeros(deg,1) Kee]);

% Add tip mass effects
%x = 1;
I_tip = m_tip * L^2;
H_tip = m_tip * L * double(subs(psi, 1));
for i = 1:deg
    for j = 1:deg
        Mee_tip(i,j) = m_tip*psi(i)*psi(j);
    end
end
M_tip = double(subs([I_tip, H_tip; H_tip', Mee_tip], 1));
M = double(subs(M + M_tip));

%% Assemble state space model

n = size(M, 1);
x = L;
Bhat = [1, zeros(1,n-1)];

A = [zeros(n), eye(n); -inv(M)*Ke, zeros(n)];
B = double([zeros(n,1); inv(M)*Bhat']);

mu = 0.5;

C_hat = [L, mu*double(subs(psi))];
D = 0;

% Controller gains
Kp = 1;
Kd = 1.8;
K = [-Kp*C_hat, -Kd*C_hat];

% Controlled SS representation
Ac = [A + B*K];

double(eig(Ac))

% Bc = [Bhat';0;0;0;0];
Bc = B;
Cc = [C_hat, zeros(1,n)];

% Step response
sys_ss = ss(Ac, Bc, Cc, D);
step(sys_ss);
