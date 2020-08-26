function [Xdot, Y,varargout]= dynamics(t,X,U,aircraft,varargin)% Y== variaveis de saida, X== variaveis de estado, U== variaveis de controle
% Var arg in variavel opcional Matlab é inteligente
%   Ordem Vertor de estado adotado
% 
%    u,v,w
%    p,q,r
%    
% X=[u
%    w
%    q
%    teta
%    h
%    x 
%{moveimento longitudinal 6 primeiras}
%{movimento latero direcional}
%    v
%    phi
%    p
%    r
%    psi
%    y]
global g;


u= X(1);
w= X(2);
q_deg_s=X(3);
theta_deg =X(4);
h=X(5);
x=X(6);

v=X(7);
phi_deg=X(8);
p_deg_s=X(9);
r_deg_s=X(10);
psi_deg=X(11);
y= X(12);

V_b=[u; v; w];
omega_b = [p_deg_s*pi/180; q_deg_s*pi/180; r_deg_s*pi/180];

C_phi = DCM(1,phi_deg*pi/180);
C_theta = DCM(2,theta_deg*pi/180);
C_psi = DCM(3,psi_deg*pi/180);
C_bv=C_phi*C_theta*C_psi;

g_b=C_bv*[0;0; g];

m = aircraft.mass.m;
r_C_b = aircraft.mass.r_C_b;
J_O_b = aircraft.mass.J_O_b;

Mgen =[m*eye(3) -m*skew(r_C_b)
    m*skew(r_C_b) J_O_b];

if length(varargin)==1 % Quantidade de argumentos a mais na função
    Vw_v = varargin{1};
% Já é conhecido Vw_v vento vertical local
    Vw_b = C_bv*Vw_v;
    Vrel_b = V_b - Vw_b;
    urel= Vrel_b(1);
    vrel= Vrel_b(2);
    wrel= Vrel_b(3);
    
    Xaero= X;

    Xaero(1)=urel;
    Xaero(2)=wrel;
    Xaero(7)=vrel;
    
    varargout{1}=Vw_b;
else
    Xaero=X;
end

[F_aero_b,M_aero_O_b,Y_aero] = aero_loads(Xaero,U,aircraft);
[F_prop_b,M_prop_O_b,Y_prop] = prop_loads(Xaero,U,aircraft);

RHS1 = F_aero_b+F_prop_b + m*g_b +...
    -m*skew(omega_b)*V_b - m*skew(omega_b)*skew(omega_b)*r_C_b;

RHS2= M_aero_O_b + M_prop_O_b + m*skew(r_C_b)*g_b + ...
    -skew(omega_b)*J_O_b*omega_b + ...
    -m*skew(r_C_b)*skew(omega_b)*V_b;

RHS=[RHS1;RHS2];

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

n_C_b = -1/(m*g)*(F_aero_b+F_prop_b);

r_P_b= aircraft.geom.r_pilot_b; % posicao do piloto

n_P_b= n_C_b + ...
    -1/g*(skew(omega_b_dot)*(r_P_b-r_C_b)+ ...
    skew(omega_b)*(skew(omega_b)*(r_P_b-r_C_b)));

Xdot = [V_b_dot(1)
        V_b_dot(3)
        q_dot_deg_s2
        theta_dot_deg_s
        hdot
        xdot
        V_b_dot(2)
        phi_dot_deg_s
        p_dot_deg_s2
        r_dot_deg_s2
        psi_dot_deg_s2
        ydot
        Y_prop(1)];
    
Y = [ Y_aero
      Y_prop(2:end)
      n_C_b
      n_P_b];
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