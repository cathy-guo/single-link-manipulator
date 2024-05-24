function ref = sin_ref_signal_up(t_array, L)

    ref.theta = []; ref.theta_d_dd = []; ref.rho_d = [];
    ref.rho_d_dot = []; ref.rho_d_dd = [];

    for i = 1:length(t_array)

        t = t_array(i);

        if t<(5*pi)
            % Define the desired output
            ref.theta(end+1) = - cos(t/5) + 1;
            ref.theta_d_dd(end+1) = 1/25 * cos(t/5);
            ref.rho_d(end+1) = L * ref.theta(end);
            ref.rho_d_dot(end+1) = L/5 * sin(t/5);
            ref.rho_d_dd(end+1) = L * ref.theta_d_dd(end);
        else
            ref.theta(end+1) = 2;
            ref.theta_d_dd(end+1) = 0;
            ref.rho_d(end+1) = L * ref.theta(end);
            ref.rho_d_dot(end+1) = 0;
            ref.rho_d_dd(end+1) = 0;
        end
    end
end