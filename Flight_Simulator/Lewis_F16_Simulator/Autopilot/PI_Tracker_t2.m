function J= PI_Tracker_t2(K,Aa,Ba,Ca,Fa,Ga,Ha,Q,R,V)

Ac = Aa -Ba*K*Ca;
Bc = Ga-Ba*K*Fa;

c=max(real(eig(Ac)));

tol =1e-3;

if c<=-tol
P0 = lyap(Ac.',Q);
P1= lyap(Ac.',P0);
P= lyap (Ac.',2*P1+Ca.'*K.'*R*K*Ca);

r0 = ones(size(Bc,2),1);

xabar = -Ac\(Bc*r0);
zbar =Ha*xabar;
ebar = r0-zbar;

J = 1/2*xabar.'*P*xabar + 0.5*ebar.'*V*ebar;

else
    J=1e10*(c+tol);
end