function [Xdot,Y,U,Ucom,Upilot,Ucas,rc,e] = ...
    nzCAS(t,X,U_eq,aircraft,X_eq,Y_eq)

tau_e=1/20.2;
tau_a=1/20.2;
tau_r=1/20.2;

delta_e_deg=X(14);
delta_a_deg=X(15);
delta_r_deg=X(16);

xi = X(17);

%----------------------------------------------------------------------------------------------------
%Pilot input
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
% if t>t0 && t<=t0+dtp
% u_r_deg= u_r_deg + dp_deg;
% elseif t>t0+dtp && t<= t0+dtp+dtn
%     u_r_deg=u_r_deg+dn_deg;
% end

Upilot = [ u_throttle
    u_e_deg
    u_a_deg
    u_r_deg];

%------------------------------------------------------------------------------------
% Dynamics

U = [u_throttle
    delta_e_deg
    delta_a_deg
    delta_r_deg];

[Xdot,Y]= dynamics(t,X(1:13),U,aircraft);

%----------------------------------------------------------------------------------------------------
%nz CAS:

z = 0.9;
k_q=-0.4;
k_p=-5.0;

q_deg_s= X(3);
q_deg_s_eq =X_eq(3);

t0 = 1;
dt_pulse = 10; % Duracao do pulso
delta_nzP_com = 0;  %Variacao Nz comandado em torno do equilibrio

nzP = Y(end);
nzP_eq= Y_eq(end);
if t>t0 && t<=t0+dt_pulse
        delta_nzP_com = 1;
elseif t>t0+dt_pulse && t<=t0+2*dt_pulse
    delta_nzP_com = -1;
% elseif t>t0+2*dt_pulse && t<=t0+3*dt_pulse
%     delta_nzP_com = 1;
end

rc= delta_nzP_com;

e = rc - (nzP-nzP_eq);

u_e_CAS= -k_q*(q_deg_s-q_deg_s_eq) + ...
    k_p*e + ...
    +k_p*z*xi;

Ucas = [0
    u_e_CAS
    0
    0];

%--------------------------------------------------------------------------------------
% Actuator dynamics

Ucom = Upilot+Ucas;

u_e_deg= Ucom(2);
u_a_deg= Ucom(3);
u_r_deg= Ucom(4);

delta_e_dot_deg_s = 1/tau_e*(u_e_deg-delta_e_deg);
delta_a_dot_deg_s = 1/tau_a*(u_a_deg-delta_a_deg);
delta_r_dot_deg_s = 1/tau_r*(u_r_deg-delta_r_deg);

xidot = e;

Xdot=[Xdot
     delta_e_dot_deg_s
     delta_a_dot_deg_s
     delta_r_dot_deg_s
     xidot];
 
 
