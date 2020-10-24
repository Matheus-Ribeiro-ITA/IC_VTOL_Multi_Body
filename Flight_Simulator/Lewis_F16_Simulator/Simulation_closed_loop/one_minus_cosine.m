function [Xdot,Y,Vw_v] = one_minus_cosine(t,X,U_eq,aircraft,Vw_v_eq)

dVw_v = zeros(3,1);

L= 5000;
x0=1000;
W0 = -50;
x=X(6);

if x>=x0 && x<=x0+L
    dVw_v(3) = W0/2*(1-cos(2*pi*(x-x0)/L));
end
Vw_v = Vw_v_eq+ dVw_v;

U= U_eq;


[Xdot, Y]= dynamics(t,X,U,aircraft,Vw_v);
