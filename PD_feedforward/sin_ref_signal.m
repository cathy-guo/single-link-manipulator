function ref = sin_ref_signal(t, L)

    % Define the desired output
    ref.theta = sin(t);

    % Find the corresponding desired angular acceleration
    ref.theta_dd = -sin(t);
    ref.rho = L * ref.theta;
    ref.rho_dot = L * cos(t);
    
end