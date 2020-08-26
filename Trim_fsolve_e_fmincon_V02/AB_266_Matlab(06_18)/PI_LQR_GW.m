function J= PI_LQR_GW(K,A,B,C,Q,R,GW)

Ac = A -B*K*C;

P= lyap (Ac.',Q+C.'*K.'*R*K*C);

J = 1/2*trace(P)+ sum(sum(GW.*K.*K));