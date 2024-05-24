function ref = spline_ref_signal(t, L)

    % Define waypoints
    xx = [0, 2, 10, 15, 20];
    yy = [0, 0, pi/2, pi/2, 1];

    % Get spline and desired rho + corresponding theta
    pp = spline(xx, yy);
    ref.rho_d = L * ppval(pp, t);
    ref.theta = ppval(pp, t);

    % Find the corresponding desired angular velocity
    f_theta_dot = fnder(pp, 1);
    ref.theta_d_dot = ppval(f_theta_dot, t);
    ref.rho_d_dot = L * ref.theta_d_dot;

    % Find the corresponding desired angular acceleration
    f_theta_dd = fnder(pp, 2);
    ref.theta_d_dd = ppval(f_theta_dd, t);
    ref.rho_d_dd = L * ref.theta_d_dd;
    
end