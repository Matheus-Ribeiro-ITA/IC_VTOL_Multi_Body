function J= PI_LQR(K,A,B,C,Q,R)

Ac = A -B*K*C;

P= lyap (Ac.',Q+C.'*K.'*R*K*C);

J = 1/2*trace(P);