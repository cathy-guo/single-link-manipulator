% Define parameters for single-link flexible model
function params = get_link_params(L)

    params.link_length = L;        % link length [m]
    params.EI = 5.41;              % stiffness 
    params.sig = 0.2332;         % mass density [kg\m]
    params.deg = 3;                % number of assumed mode shapes
    params.mt = 0*params.sig*L;
    params.Ihub = 0.005;

end