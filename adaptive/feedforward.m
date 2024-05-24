function U = feedforward(ref, I, Kp, Kd, X, C_hat)
% Returns the control input for a feedforward + PD feedback control scheme
    n = size(C_hat, 2);
    rho_mu = C_hat*X(1:n);
    rho_mu_dot = C_hat*X(n+1:2*n);
    U = I*ref.theta_d_dd - Kp*(rho_mu - ref.rho_d) - Kd*(rho_mu_dot - ref.rho_d_dot);
end