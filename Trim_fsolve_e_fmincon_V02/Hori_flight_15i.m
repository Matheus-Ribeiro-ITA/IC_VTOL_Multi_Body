function [Res,varargout] = ...
    Hori_flight_15i(t,X,Xp,U_eq,aircraft,X_eq,Y_eq)

tau_t=1/20.2;
tau_e=1/20.2;
tau_a=1/20.2;
tau_r=1/20.2;

delta_t=X(15);
delta_e=X(16);
delta_a=X(17);
delta_r=X(18);


%----------------------------------------------------------------------------------------------------
%Pilot input

t0=1; % Passa um segundo no zero
dp_deg= 4; % angulo pulso positivo
dtp=1; % Duração pulso positivo
dn_deg=-4;
dtn=1;



% Throttle
u_throttle = U_eq(1);

% Elevator Doublet:
u_e_deg= U_eq(2);

if t>t0 && t<=t0+dtp
u_e_deg= u_e_deg + dp_deg;

elseif t>t0+dtp && t<= t0+dtp+dtn
    u_e_deg=u_e_deg+dn_deg;
end

%Aileron
u_a_deg= U_eq(3);

%Rudder
u_r_deg=U_eq(4);

% end

Upilot = [ u_throttle
    u_e_deg
    u_a_deg
    u_r_deg];
%------------------------------------------------------------------------------------
% Dynamics

U = [delta_t
    delta_e
    delta_a
    delta_r];


[Res_din, Y]= dynamics(0,X(1:14),Xp(1:14),U,aircraft);


%----------------------------------------------------------------------------------------------------
%SAS:

% k_hight= -5;
% 
% 
% 
% u_e_SAS= k_hight*(X(3)-X_eq(3));
% u_t_SAS= -0.1*k_hight*(X(3)-X_eq(3));


k_alpha = -0.6;
k_q = -0.2;

alpha_deg = Y(8);
q_deg_s= X(11)*180/pi;

alpha_deg_eq =Y_eq(8);
q_deg_s_eq =X_eq(11)*180/pi;

u_e_SAS = -k_alpha*(alpha_deg-alpha_deg_eq)+ ...
    -k_q*(q_deg_s-q_deg_s_eq);


Usas = [0
    u_e_SAS 
    0
    0];

%--------------------------------------------------------------------------------------
% Actuator dynamics

Ucom = Upilot+Usas;


u_t    = Ucom(1);
u_e_deg= Ucom(2);
u_a_deg= Ucom(3);
u_r_deg= Ucom(4);

delta_t_dot = 1/tau_t*(u_t-delta_t);
delta_e_dot_deg_s = 1/tau_e*(u_e_deg-delta_e);
delta_a_dot_deg_s = 1/tau_a*(u_a_deg-delta_a);
delta_r_dot_deg_s = 1/tau_r*(u_r_deg-delta_r);


Res=[Res_din
     delta_t_dot - Xp(15)
     delta_e_dot_deg_s - Xp(16)
     delta_a_dot_deg_s - Xp(17)
     delta_r_dot_deg_s - Xp(18)];
 
varargout{1} = [Y
                Ucom];
 
 
