function [F_prop_b,M_prop_O_b,Y_prop] = prop_loads(X,U,aircraft)


power = U(1);

u = X(4);
w= X(5);
v=X(6);
h=X(3);
p=X(10);
r=X(12);
q=X(11);

sigma=X(13);

V = sqrt(u^2+v^2+w^2);
% Thrust:
% [~,~,~,a]= ISA(h);
% M = V/a;

[T,M_y] = thrust(power,sigma,aircraft); % N

% No sistema do corpo:


F_prop_b = DCM_original(2,-sigma)*[T; 0; 0];

omega_b = [p; q; r];
hex = 0;
M_prop_O_b = -skew(omega_b)*[hex; 0; 0];

M_prop_O_b = [0; M_y; 0];

Y_prop = [T
    power];

end


function [T,M_y] = thrust(pow,sigma,aircraft)

T= 4*1.5*9.8*pow;

M_y= -T/2*cos(sigma)*(aircraft.r_r{1}(3))-T/2*cos(sigma)*(aircraft.r_r{3}(3)) ...
    -T/2*sin(sigma)*(aircraft.r_r{1}(1)-0.09)-T/2*sin(sigma)*(aircraft.r_r{3}(1)-0.09);
% subtrai 0.07 da rotação do motor
end