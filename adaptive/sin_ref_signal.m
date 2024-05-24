function ref = sin_ref_signal(t, L)

    % Define the desired output
    ref.theta = sin(t);

    % Find the corresponding desired angular acceleration
    ref.theta_d_dd = -sin(t);
    ref.rho_d = L * ref.theta;
    ref.rho_d_dot = L * cos(t);
    ref.rho_d_dd = L * ref.theta_d_dd;
    
end