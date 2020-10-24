% -------------------------------------------------------
% Uni Body Dynamics
% -------------------------------------------------------
% Author: Matheus Ribeiro T-21
% gmail: matehus.ribeiro.aer@gmail.com
% Version: 0.0.1 
% Description: Uni body aircraft dynamics.
% -------------------------------------------------------


function [Xdot, Y,varargout]= dynamics_uni_body(t,X,U,aircraft,varargin)% Y== variaveis de saida, X== variaveis de estado, U== variaveis de controle

global g;


x=X(1);
y=X(2);
h=X(3);

u= X(4);
v= X(5);
w= X(6);

phi_deg=X(7)*180/pi;
theta_deg =X(8)*180/pi;
psi_deg=X(9)*180/pi;

p_deg_s=X(10)*180/pi;
q_deg_s=X(11)*180/pi;
r_deg_s=X(12)*180/pi;

sigma=X(13);
sigma_dot=X(14);



V_b=[u; v; w];
omega_b = [p_deg_s*pi/180; q_deg_s*pi/180; r_deg_s*pi/180];

C_phi = DCM_original(1,phi_deg*pi/180);
C_theta = DCM_original(2,theta_deg*pi/180);
C_psi = DCM_original(3,psi_deg*pi/180);
C_bv=C_phi*C_theta*C_psi;

g_b=C_bv*g;

m = aircraft.m_total;

% r_0=aircraft.r_b_0*aircraft.m_B;
% 
% for i=1:aircraft.n_rotor
% r_r_pivot{i}=DCM_original(2,sigma)*(aircraft.r_r_pivot{i}); % Gira o pivot
% r_r_0{i} = r_r_pivot{i} + aircraft.r_pivot_0{i}; % Muda pra ref. inercial
% r_0 = r_0 + r_r_0{i}*aircraft.m_r{i};
% end
% 
% r_0= r_0/aircraft.m_total;


% r_C_b = r_0-aircraft.r_b_0;
r_C_b = [0;0;0];
J_O_b = aircraft.I_total;

Mgen =[m*eye(3) -m*skew(r_C_b)
    m*skew(r_C_b) J_O_b];


    Xaero=X;


[F_aero_b,M_aero_O_b,Y_aero] = aero_loads_cg(Xaero,U,aircraft);
[F_prop_b,M_prop_O_b,Y_prop] = prop_loads_cg(Xaero,U,aircraft);

% F_aero_b(3,1) = -F_aero_b(3,1);
% F_prop_b(3,1) = -F_prop_b(3,1);

RHS1 = F_aero_b+F_prop_b + m*g_b +...
    -m*skew(omega_b)*V_b - m*skew(omega_b)*skew(omega_b)*r_C_b;

RHS2= M_aero_O_b + M_prop_O_b + m*skew(r_C_b)*g_b + ...
    -skew(omega_b)*J_O_b*omega_b + ...
    -m*skew(r_C_b)*skew(omega_b)*V_b;

RHS=[RHS1;RHS2];

% F_z=F_aero_b(3,1)
% Peso=m*g_b(3,1)
% F_aero_b+F_prop_b + m*g_b
% RHS
Vomegadot = Mgen\RHS;

V_b_dot=Vomegadot(1:3);
omega_b_dot= Vomegadot (4:6);

C_vb = C_bv.';
dREOdt= C_vb*V_b;
xdot= dREOdt(1);
ydot= dREOdt(2);
hdot= -dREOdt(3);

I3 = eye(3);
e31= I3(:,1);
e32= I3(:,2);
e33= I3(:,3);

K_Phi = [e31 C_phi*e32 C_bv*e33];
Phi_dot = K_Phi\omega_b; %rad/s

q_dot_deg_s2 = omega_b_dot(2)*180/pi;
p_dot_deg_s2 = omega_b_dot(1)*180/pi;
r_dot_deg_s2 = omega_b_dot(3)*180/pi;

theta_dot_deg_s = Phi_dot(2)*180/pi;
phi_dot_deg_s = Phi_dot(1)*180/pi;
psi_dot_deg_s2 = Phi_dot(3)*180/pi;

n_C_b = -1/(m*g(3,1))*(F_aero_b+F_prop_b);

% r_P_b= aircraft.geom.r_pilot_b; % posicao do piloto
% 
% n_P_b= n_C_b + ...
%     -1/g*(skew(omega_b_dot)*(r_P_b-r_C_b)+ ...
%     skew(omega_b)*(skew(omega_b)*(r_P_b-r_C_b)));

Xdot = [xdot
        ydot
        hdot
        V_b_dot(1)
        V_b_dot(2)
        V_b_dot(3)
        phi_dot_deg_s*pi/180
        theta_dot_deg_s*pi/180
        psi_dot_deg_s2*pi/180
        p_dot_deg_s2*pi/180
        q_dot_deg_s2*pi/180
        r_dot_deg_s2*pi/180 
        0
        0];
    
Y = [ Y_aero
      Y_prop(2:end)
      F_aero_b
      F_prop_b];
%   
%   Y_aero = [V
%     alpha_deg
%     beta_deg
%     CX
%     CY
%     CZ
%     Cl
%     Cm
%     Cn
%     CD
%     CS % Coeficiente de força lateral
%     CL];
%    