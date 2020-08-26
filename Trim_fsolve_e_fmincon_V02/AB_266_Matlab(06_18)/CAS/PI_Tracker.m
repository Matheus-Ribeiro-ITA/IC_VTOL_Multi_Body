function J= PI_Tracker(K,Aa,Ba,Ca,Fa,Ga,Ha,Q,R,V)

Ac = Aa -Ba*K*Ca;
Bc = Ga-Ba*K*Fa;

P= lyap (Ac.',Q+Ca.'*K.'*R*K*Ca);

r0 = ones(size(Bc,2),1);

xabar = -Ac\(Bc*r0);
zbar =Ha*xabar;
ebar = r0-zbar;

J = 1/2*xabar.'*P*xabar + 0.5*ebar.'*V*ebar;