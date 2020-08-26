function [Xdot,Y,U,Ucom] = control_input(t,X,U_eq,aircraft)


tau_e=1/20.2;
tau_a=1/20.2;
tau_r=1/20.2;


delta_e_deg=X(14);
delta_a_deg=X(15);
delta_r_deg=X(16);


t0=1; % Passa um segundo no zero
dp_deg= 3; % angulo pulso positivo
dtp=1; % Duração pulso positivo
dn_deg=-3;
dtn=1;

% Throttle
u_throttle = U_eq(1);

% Elevator Doublet:
u_e_deg= U_eq(2);

% if t>t0 && t<=t0+dtp
% u_e_deg= u_e_deg + dp_deg;
% 
% elseif t>t0+dtp && t<= t0+dtp+dtn
%     u_e_deg=u_e_deg+dn_deg;
% end

%Aileron
u_a_deg= U_eq(3);

%Rudder
u_r_deg=U_eq(4);

if t>t0 && t<=t0+dtp
u_r_deg= u_r_deg + dp_deg;

elseif t>t0+dtp && t<= t0+dtp+dtn
    u_r_deg=u_r_deg+dn_deg;
end



%---------------------------------------------------
U = [u_throttle
    delta_e_deg
    delta_a_deg
    delta_r_deg];

[Xdot,Y]= dynamics(t,X(1:13),U,aircraft);

delta_e_dot_deg_s = 1/tau_e*(u_e_deg-delta_e_deg);
delta_a_dot_deg_s = 1/tau_a*(u_a_deg-delta_a_deg);
delta_r_dot_deg_s = 1/tau_r*(u_r_deg-delta_r_deg);


Xdot=[Xdot
     delta_e_dot_deg_s
     delta_a_dot_deg_s
     delta_r_dot_deg_s];
 
 Ucom =[u_throttle
        u_e_deg
        u_a_deg
        u_r_deg];
