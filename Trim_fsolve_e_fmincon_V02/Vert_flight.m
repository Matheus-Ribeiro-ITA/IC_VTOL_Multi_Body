function [Xdot,Y,U,Ucom,Upilot,Usas] = ...
    Vert_flight(t,X,U_eq,aircraft,X_eq,Y_eq)

tau_t=1/20.2;
tau_e=1/20.2;
tau_a=1/20.2;
tau_r=1/20.2;

delta_t=X(15);
delta_e=X(16);
delta_a=X(17);
delta_r=X(18);

% -------------------------------------------------------------------------------------------
% % One-minus cossine
% 
% dVw_v = zeros(3,1);
% L= 450;
% x0=150;
% W0 = -20;
% x=X(6);
% if x>=x0 && x<=x0+L
%     dVw_v(3) = W0/2*(1-cos(2*pi*(x-x0)/L));
% end
% Vw_v = Vw_v_eq+ 0*dVw_v; %%%%%%%%

%----------------------------------------------------------------------------------------------------
%Pilot input
% t0=1; % Passa um segundo no zero
% dp_deg= 3; % angulo pulso positivo
% dtp=1; % Duração pulso positivo
% dn_deg=-3;
% dtn=1;

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

U = [delta_t
    delta_e
    delta_a
    delta_r];



t0= 0;
y0= X(1:14);
fixed_y0= ones(1,14);
 
yp0= zeros(1,14);
% yp0(1)=trim_par.V;
fixed_yp0 = [];


[X,X_dot] = decic(@dynamics,t0,y0,fixed_y0,yp0,fixed_yp0,[],U,aircraft);
[~, Y]= dynamics(0,X,X_dot,U,aircraft);

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


Xdot=[X_dot
     delta_t_dot
     delta_e_dot_deg_s
     delta_a_dot_deg_s
     delta_r_dot_deg_s];
 
 
