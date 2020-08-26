function [c,ceq] = max_eig_real(K,A,B,C)

Ac= A-B*K*C;

tol =1e-3;

c = max(real(eig(Ac))) + tol;

ceq=0;