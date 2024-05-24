function [L, EI, sig, deg, Ke, M_link, M_tip, M, psi, I_link, I] = get_params(m_tip)

%% Import model params, and define EoMs for beam
L = 1;
EI = 1;
sig = 1;
deg = 3;

% Form psi basis funcs for rayleigh ritz
syms x;
for i = 1:deg
    psi(i) = x^(i+1);
end

% Beam moment of inertia (= 1/3)
I_link = sig*int(x^2, 0, L);

% Formulate mass matrix for beam
for i = 1:deg
    H(i,1) = sig*int(x*psi(i),0,L);
    for j = 1:deg
        Mee(i,j) = sig*int(psi(i)*psi(j), 0, L);
    end
end
M_link = double([I_link H'; H Mee]);

% Formulate stiffness matrix
for i = 1:deg
    for j = 1:deg
        Kee(i,j) = EI * int(diff(psi(i), 2)* diff(psi(j),2), 0, L);
    end
end
Ke = double([0 zeros(1,deg); zeros(deg,1) Kee]);

% Add tip mass effects
I_tip = m_tip * L^2;
for i = 1:deg
    H_tip(i) = m_tip * L*psi(i);
    for j = 1:deg
        Mee_tip(i,j) = m_tip*psi(i)*psi(j);
    end
end
H_tip = subs(H_tip,L);
Mee_tip = subs(Mee_tip, L);
M_tip = double([I_tip, H_tip; H_tip', Mee_tip]);

M = double(M_link + M_tip);

I = double(subs(I_link, L)) + I_tip;

end