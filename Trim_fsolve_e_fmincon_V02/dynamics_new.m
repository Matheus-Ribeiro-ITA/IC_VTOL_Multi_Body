function [Res,varargout]= dynamics(t,X,Xp,U,aircraft,varargin)


global g

% x=X(1);
% y=X(2);
% h=X(3);


V=X(4:6);
V_dot=Xp(4:6);
euler=X(7:9);
w=X(10:12);
w_dot=Xp(10:12);

sigma=X(13);
sigma_dot=X(14);
sigma_dot_dot=Xp(14);

%% --------------------------------------------------------------------------------
% Preallocating for speed
% ---------------------------------------------------------------------
point{1}=0;
point{2}=0;
point{3}=0;
point{4}=0;

vec{1}=zeros(3,1);
vec{2}=zeros(3,1);
vec{3}=zeros(3,1);
vec{4}=zeros(3,1);

mat{1}=zeros(3,3);
mat{2}=zeros(3,3);
mat{3}=zeros(3,3);
mat{4}=zeros(3,3);


m_r=point;

r_pivot=vec;
I_r=vec;
w_r=vec;
w_dot_r=vec;
V_r=vec;
a_r=vec;
r_r_pivot=vec;
r_r=vec;

R_r=mat;
R_dot_r=mat;
R_dot_dot_r=mat;
T_r=mat;
T_dot_r=mat;
T_dot_dot_r=mat;
R_T_r=mat;
R_dot_T_r=mat;
%% --------------------------------------------------------------------------------
% Read Struct
% ---------------------------------------------------------------------

n_rotor= aircraft.n_rotor;
m_B=aircraft.m_B;
r_b_0=aircraft.r_b_0;
I_B = aircraft.I_B;


for i=1:n_rotor
    r_pivot{i}=aircraft.r_pivot{i};
    I_r{i}=aircraft.I_r{i};
    m_r{i}=aircraft.m_r{i};
end

%% --------------------------------------------------------------------------------
% Rotation Matrix
% ---------------------------------------------------------------------
% Omega_B
omega_B=skew(w);
omega_B_dot=skew(w_dot);


C_phi = DCM_original(1,euler(1,1));
C_theta = DCM_original(2,euler(2,1));
C_psi = DCM_original(3,euler(3,1));
C_bv=C_phi*C_theta*C_psi;


for i=1:n_rotor
R_r{i}=[cos(sigma) 0 sin(sigma)
        0          1       0
        -sin(sigma) 0  cos(sigma)]; % 
end


for i=1:n_rotor
R_dot_r{i}=[-sigma_dot*sin(sigma) 0 sigma_dot*cos(sigma)
        0          0       0
        -sigma_dot*cos(sigma) 0  -sigma_dot*sin(sigma)]; % 
end

for i=1:n_rotor
R_dot_dot_r{i}=[-(sigma_dot_dot*sin(sigma)+sigma_dot*cos(sigma)) 0 (sigma_dot_dot*cos(sigma)-sigma_dot*sin(sigma))
        0          0       0
        -(sigma_dot_dot*cos(sigma)-sigma_dot*sin(sigma)) 0  -(sigma_dot_dot*sin(sigma)+sigma_dot*cos(sigma))]; % 
end

%% --------------------------------------------------------------------------------
% Relative velocities
% ---------------------------------------------------------------------

for i=1:n_rotor
w_r{i}=[0;sigma_dot;0];
w_dot_r{i}=[0;sigma_dot_dot;0];
end

for i=1:n_rotor
V_r{i}= R_dot_r{i}*aircraft.r_r_pivot{i};
end

for i=1:n_rotor
a_r{i}= R_dot_dot_r{i}*aircraft.r_r_pivot{i};
end

%% --------------------------------------------------------------------------------
% Rotation Inertia Matrix
% ---------------------------------------------------------------------



for i=1:n_rotor
T_r{i}=[cos(sigma) 0 sin(sigma) 
        0 1 0 
        -sin(sigma) 0 cos(sigma)];
    
T_dot_r{i}=[-sigma_dot*sin(sigma) 0 sigma_dot*cos(sigma)
        0 0 0 
        -sigma_dot*cos(sigma) 0 -sigma_dot*sin(sigma)];
    
T_dot_dot_r{i}=[-sigma_dot_dot*sin(sigma)-(sigma_dot^2)*cos(sigma) 0 sigma_dot_dot*cos(sigma)-(sigma_dot^2)*sin(sigma)
        0 0 0 
        -sigma_dot_dot*cos(sigma)+(sigma_dot^2)*sin(sigma) 0 -sigma_dot_dot*sin(sigma)-(sigma_dot^2)*cos(sigma)];
end

%% --------------------------------------------------------------------------------
% Translation Inertia Matrix
% ---------------------------------------------------------------------

for i=1:n_rotor
r_r_pivot{i}=DCM_original(2,-sigma)*aircraft.r_r_pivot{i}; % Gira o pivot
r_r{i} = r_r_pivot{i} + r_pivot{i}; % Muda pra ref. body
end

for i=1:n_rotor
R_T_r{i}=[r_r{i}(2)^2+r_r{i}(3)^2   -r_r{i}(1)*r_r{i}(2)   -r_r{i}(1)*r_r{i}(3)
        -r_r{i}(1)*r_r{i}(2)   r_r{i}(1)^2+r_r{i}(3)^2   -r_r{i}(2)*r_r{i}(3)         
         -r_r{i}(1)*r_r{i}(3)   -r_r{i}(2)*r_r{i}(3)    r_r{i}(1)^2+r_r{i}(2)^2];
end


for i=1:n_rotor
R_dot_T_r{i}=[2*(r_r{i}(2)*V_r{i}(2)+r_r{i}(3)*V_r{i}(3))   -(V_r{i}(1)*r_r{i}(2)+r_r{i}(1)*V_r{i}(2))   -(V_r{i}(1)*r_r{i}(3)+r_r{i}(1)*V_r{i}(3))   
        -(V_r{i}(1)*r_r{i}(2)+r_r{i}(1)*V_r{i}(2))   2*(r_r{i}(1)*V_r{i}(1)+r_r{i}(3)*V_r{i}(3))   -(V_r{i}(2)*r_r{i}(3)+r_r{i}(2)*V_r{i}(3))         
        -(V_r{i}(1)*r_r{i}(3)+r_r{i}(1)*V_r{i}(3))   -(V_r{i}(2)*r_r{i}(3)+r_r{i}(2)*V_r{i}(3))    2*(r_r{i}(1)*V_r{i}(1)+r_r{i}(2)*V_r{i}(2))];
end








%% --------------------------------------------------------------------------------
%---------------------------------------------------------------------------------

% PART 2

%---------------------------------------------------------------------
%---------------------------------------------------------------------


%% --------------------------------------------------------------------------------
% ANGULAR MOTION
% ----------------------------------------------------------------------------
%% A calculation


A=I_B;


% IB_til = T_r{i}*aircraft.I_r{i}*T_r{i}' + aircraft.m_r{i}*R_T_r{i} 
for i=1:n_rotor
    A= A+ T_r{i}*I_r{i}*T_r{i}' + m_r{i}*R_T_r{i} ; 
end




%% B calculation

B=omega_B*I_B;


for i=1:n_rotor
    B=B + T_dot_r{i}*I_r{i}*T_r{i}' ...
        + T_r{i}*I_r{i}*T_dot_r{i}' ...
        + m_r{i}*R_dot_T_r{i} ...
        + omega_B*( T_r{i}*I_r{i}*T_r{i}' + m_r{i}*R_T_r{i} );
end


%% C calculation
C= zeros(3,3);


for i=1:n_rotor
    C= C + m_r{i}*skew(r_r{i});
end

%% D calculation

D= zeros(3,3);


for i=1:n_rotor
    D= D + m_r{i}*( skew(R_dot_r{i}*r_r_pivot{i}) ...
        + skew(r_r{i})*omega_B );
end

%% E calculation

E=[0
    0
    0];

for i=1:n_rotor
    
    E=E +( T_dot_r{i}*aircraft.I_r{i}*T_r{i}' ...
        + T_r{i}*aircraft.I_r{i}*T_dot_r{i}' ...
        + aircraft.m_r{i}*R_dot_T_r{i} ...
        + omega_B*( T_r{i}*aircraft.I_r{i}*T_r{i}' + aircraft.m_r{i}*R_T_r{i} )...
        )*R_r{i}*w_r{i} ... % 1º Termo
        + (T_r{i}*aircraft.I_r{i}*T_r{i}' + aircraft.m_r{i}*R_T_r{i})...
        *(T_dot_r{i}*w_r{i} + T_r{i}*w_dot_r{i}) ... % 2º Termo
        + m_r{i}*( skew(T_dot_r{i}*aircraft.r_r_pivot{i})*...
        (omega_B*r_r{i})+T_dot_r{i}*aircraft.r_r_pivot{i}) ...
        + skew(r_r{i})*((omega_B_dot+omega_B*omega_B)*r_r{i} +(2*omega_B*T_dot_r{i}+T_dot_dot_r{i})*aircraft.r_r_pivot{i} );
end

%%  Mp calculation

Mp=skew(r_b_0)*m_B*C_bv*g;
   
for i=1:n_rotor
    Mp=Mp+ skew(r_r{i})*m_r{i}*C_bv*g;
end

%% Aerodynamic torques on body
% Torque_B =[0
%            0
%            0];
[F_B_aero, M_B_aero, Y_aero] = aero_loads_cg(X,U,aircraft);   
[F_B_prop,M_B_prop, ~] = prop_loads_cg(X,U,aircraft);


Torque_B= M_B_aero+M_B_prop;
       
%% Angular aceleration


w_dot_02=  A\(Torque_B +Mp - B*w - C*V_dot - D*V -E);

% D
% V
% D_V=D*V

%% --------------------------------------------------------------------------------
% TRANSLATION MOTION
% ---------------------------------------------------------------------

% Aerodynamic forces on body
% F_B=[0
%     0
%     0];

% [F_B_aero ,~, ~] = aero_loads(X,U,aircraft); 
% [F_B_prop ,~, ~] = prop_loads(X,U,aircraft);

F_B = F_B_prop+ F_B_aero;

F=[0
   0
   0];


for i=1:aircraft.n_wing
    F = F ...
        + aircraft.m_r{i}*((skew(w_dot) + skew(w)*skew(w))*aircraft.r_r{i} ...
        + 2*skew(w)*V_r{i} + a_r{i});
end




V_dot_02=-omega_B*V+F_B/aircraft.m_total + C_bv*g - F;


  
%% Earth frame

V_e=(C_bv^-1)*V;


%% Euler Angle Kinematics

H= [1 sin(euler(1))*tan(euler(2)) cos(euler(1))*tan(euler(2))
    0 cos(euler(1)) -sin(euler(1)) 
    0 sin(euler(1))/cos(euler(2)) cos(euler(1))*tan(euler(2))]; 

euler_p=H*w;


%% Rotors Rotation

% sigma_dot_2= Xp(13);
% sigma_dot_dot_2= Xp(14);

%%

Res=[Xp(1)-V_e(1);
    Xp(2)-V_e(2);
    Xp(3)-V_e(3); 
    V_dot-V_dot_02;
    Xp(7)-euler_p(1);
    Xp(8)-euler_p(2);
    Xp(9)-euler_p(3);
    w_dot_02-w_dot; %10,11,,12
    X(14)-Xp(13);
    Xp(14)];

varargout{1} = [F_B
            Torque_B
            Y_aero(1:3,1) % 7= V ,8= alpha,9= Beta
            Y_aero(12,1)  % 10= CL
            Y_aero(8,1)]; % 11= Cm
            
             

