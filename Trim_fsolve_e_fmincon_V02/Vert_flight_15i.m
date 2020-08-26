function [Res,varargout] = ...
    Vert_flight_15i(t,X,Xp,U_eq,aircraft,X_eq)

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

% Throttle
u_throttle = U_eq(1);

% Elevator Doublet:
u_e_deg= U_eq(2);

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

k_hight= -1;



u_t_SAS= k_hight*(X(3)-X_eq(3));


Usas = [u_t_SAS
    0
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
     delta_r_dot_deg_s - Xp(18)]
 
 
