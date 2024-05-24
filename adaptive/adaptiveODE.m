function Xdot = adaptiveODE(t, X, n, A, B, C_hat, L, I_link, Kd, Lambda, Gamma, ref)
% ODE function for the single link case with an adaptive payload-mass
% estimate
% X = [q, q_dot, a_hat]
    
    % Get reference signal values
    reference   = ref(t, L);
    rho_d       = reference.rho_d;
    rho_d_dot   = reference.rho_d_dot;
    theta_d_dd  = reference.theta_d_dd;
    rho_d_dd   = reference.rho_d_dd;

    % Compute feedback torque input
    rho_mu = C_hat*X(1:n);
    rho_mu_dot = C_hat*X(n+1:2*n);
    s_mu = (rho_mu_dot - rho_d_dot) + Lambda*(rho_mu - rho_d);
    tau_fb = -Kd * s_mu;
    
    % Compute feedforward torque input
    m_tip_hat = X(9,:);
    % m_tip_hat = 10;
    I_hat = m_tip_hat * L^2 + I_link; 
    tau_ff = m_tip_hat * L * (rho_d_dd - Lambda * (rho_mu_dot - rho_d_dot));   
    
    % Compute total input
    U = tau_ff + tau_fb;
    
    % Build dynamics
    Xdot = zeros(2*n+1, 1);
    Xdot(1:8,:) = A*X(1:8,:) + B*U;
    Xdot(9,:) = - Gamma * (rho_d_dd - Lambda*(rho_mu_dot - rho_d_dot)) * s_mu;
    % Xdot(9,:) = 0;

end

