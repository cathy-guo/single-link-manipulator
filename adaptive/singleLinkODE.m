function [Xdot, U] = singleLinkODE(t, X, A, B, C_hat, L, I_beam, Kp, Kd, ref_signal)
% Takes in the single link state X = [q, qdot].T, system dynamic matrices A
% and B, and outputs the time derivative Xdot = [qdot, qdotdot].T

    % Get the reference signal values
    ref = ref_signal(t, L);

    % Calculate the control inputs
    U = feedforward(ref, I_beam, Kp, Kd, X, C_hat);

    % Calculate Xdot
    Xdot = A*X + B*U;

end
