function U = feedforward(ref, I_beam, Kp, Kd, X, C_hat)
% Returns the control input for a feedforward + PD feedback control scheme

    rho_mu = [Kp*C_hat, Kd*C_hat]*X
    
    U = I_beam*ref.theta_dd - rho_mu + Kp*ref.rho + Kd*ref.rho_dot;

end