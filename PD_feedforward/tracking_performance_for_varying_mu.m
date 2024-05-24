%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot sin-wave tracking performance for varying mu values
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
    
    % Feedforward control tracking a sin wave (True payload mass)  
    Kp = 3.2;
    Kd = 4;
    tspan = [0 30];
    X0 = [zeros(1,n), 1, zeros(1, n-1)]';
    [t,X] = ode45(@(t,X) singleLinkODE(t, X, A, B, C_hat, L, I, Kp, Kd), tspan, X0);
    
    % Plot
    C = [L, double(subs(psi, L)), zeros(1,n)];
    rho = C*(X');
    subplot(2,2,k);
    plot(t, rho, 'LineWidth', 1.5); hold on;
    plot(t, L*sin(t), '--'); hold on;
    legend("\rho_\mu", "Reference")
    ylabel("\rho-tip position"); xlabel("time");
    title("Feedforward + PD Control With \mu = " + string(mu))
    grid on
end