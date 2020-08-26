function [Xdot,Y,U,Ucom,Upilot,Usas,Vw_v] = ...
    PitchSAS_wind(t,X,U_eq,aircraft,Vw_v_eq,X_eq,Y_eq)

tau_e=1/20.2;
tau_a=1/20.2;
tau_r=1/20.2;

delta_e_deg=X(14);
delta_a_deg=X(15);
delta_r_deg=X(16);

% -------------------------------------------------------------------------------------------
% One-minus cossine

dVw_v = zeros(3,1);
L= 450;
x0=150;
W0 = -20;
x=X(6);
if x>=x0 && x<=x0+L
    dVw_v(3) = W0/2*(1-cos(2*pi*(x-x0)/L));
end
Vw_v = Vw_v_eq+ dVw_v;

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
if t>t0 && t<=t0+dtp
u_r_deg= u_r_deg + dp_deg;
elseif t>t0+dtp && t<= t0+dtp+dtn
    u_r_deg=u_r_deg+dn_deg;
end

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

[Xdot,Y]= dynamics(t,X(1:13),U,aircraft,Vw_v);

%----------------------------------------------------------------------------------------------------
%SAS:

k_alpha = -0.6;
k_q = -0.2;

alpha_deg = Y(2);
q_deg_s= X(3);
alpha_deg_eq =Y_eq(2);
q_deg_s_eq =X_eq(3);

u_e_SAS = -k_alpha*(alpha_deg-alpha_deg_eq)+ ...
    -k_q*(q_deg_s-q_deg_s_eq);

Usas = [0
    u_e_SAS
    0
    0];

%--------------------------------------------------------------------------------------
% Actuator dynamics

Ucom = Upilot+Usas;

u_e_deg= Ucom(2);
u_a_deg= Ucom(3);
u_r_deg= Ucom(4);

delta_e_dot_deg_s = 1/tau_e*(u_e_deg-delta_e_deg);
delta_a_dot_deg_s = 1/tau_a*(u_a_deg-delta_a_deg);
delta_r_dot_deg_s = 1/tau_r*(u_r_deg-delta_r_deg);


Xdot=[Xdot
     delta_e_dot_deg_s
     delta_a_dot_deg_s
     delta_r_dot_deg_s];
 
 
