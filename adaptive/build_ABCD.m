function [A, B, C_hat, D] = build_ABCD(M, Ke, psi, L, n, mu)

Bhat = [1, zeros(1,n-1)];
A = [zeros(n), eye(n); -inv(M)*Ke, zeros(n)];
B = double([zeros(n,1); inv(M)*Bhat']);
C_hat = [L, mu*double(subs(psi, L))];
D = 0;

end