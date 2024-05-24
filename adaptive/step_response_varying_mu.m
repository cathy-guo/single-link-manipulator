%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot step response for varying mu values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Import model params, and define EoMs for beam
L = 1;
EI = 1;
sig = 1;
deg = 3;
m_tip = 10;
mus = [0, 0.5, 0.9, 1];

% For each mu-val, plot the tracking performance
for k = 1:4
    mu = mus(k);
    
    [L, EI, sig, deg, Ke, M_link, M_tip, M, psi, I_link, I] = get_params(m_tip);
    
    % Build state space matrices
    n = size(M, 1);
    [A, B, C_hat, D] = build_ABCD(M, Ke, psi, L, n, mu);

    % Controller gains
    Kp = 1.8;
    Kd = 5;
    K = [-Kp*C_hat, -Kd*C_hat];

    % PD Controlled SS representation
    Ac = [A + B*K];
    Bc = Kp*B;
    Cc_mu = [L, mu*double(subs(psi, L)), zeros(1,n)];
    Cc_true = [L, double(subs(psi, L)), zeros(1,n)];

    % Get step response and plot    
    sys_step_mu = ss(Ac, Bc, Cc_mu, D);
    subplot(2,2,k);
    step(sys_step_mu);
    hold on;
    
    sys_step_true = ss(Ac, Bc, Cc_true, D);
    step(sys_step_true);
    hold on;
    
    title("\mu = " + string(mu))
    legend("\mu-tip position", "true tip position");
    grid on;
    
end