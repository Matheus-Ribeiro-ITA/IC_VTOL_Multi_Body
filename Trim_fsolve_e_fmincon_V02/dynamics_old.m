function [Res,varargout]= dynamics(t,X,Xp,U,aircraft,varargin)


global g

x=X(1);
y=X(2);
h=X(3);

V=X(4:6);
V_dot=Xp(4:6);
euler=X(7:9);
w=X(10:12);
w_dot=Xp(10:12);

sigma=X(13);
sigma_dot=X(14);
sigma_dot_dot=Xp(14);

for i=1:aircraft.n_wing
w_w_w{i}=[0;0;0];
w_dot_w_w{i}=[0;0;0];
end

for i=1:aircraft.n_rotor
w_r_r{i}=[0;0;0];
w_dot_r_r{i}=[0;0;0];
end




%% --------------------------------------------------------------------------------
% Rotation Matrix
% ---------------------------------------------------------------------
% Omega_B
omega_B=skew(w);


C_phi = DCM_original(1,euler(1,1));
C_theta = DCM_original(2,euler(2,1));
C_psi = DCM_original(3,euler(3,1));
C_bv=C_phi*C_theta*C_psi;


for i=1:aircraft.n_wing
R_w{i}=[cos(sigma) 0 sin(sigma)
        0          1       0
        -sin(sigma) 0  cos(sigma)]; % 
end

for i=1:aircraft.n_rotor
R_r{i}=[cos(sigma) 0 sin(sigma)
        0          1       0
        -sin(sigma) 0  cos(sigma)]; % 
end

for i=1:aircraft.n_wing
R_dot_w{i}=[-sigma_dot*sin(sigma) 0 sigma_dot*cos(sigma)
        0          0       0
        -sigma_dot*cos(sigma) 0  -sigma_dot*sin(sigma)]; % 
end


for i=1:aircraft.n_rotor
R_dot_r{i}=[-sigma_dot*sin(sigma) 0 sigma_dot*cos(sigma)
        0          0       0
        -sigma_dot*cos(sigma) 0  -sigma_dot*sin(sigma)]; % 
end

for i=1:aircraft.n_wing
R_dot_dot_w{i}=[-(sigma_dot_dot*sin(sigma)+sigma_dot*cos(sigma)) 0 (sigma_dot_dot*cos(sigma)-sigma_dot*sin(sigma))
        0          0       0
        -(sigma_dot_dot*cos(sigma)-sigma_dot*sin(sigma)) 0  -(sigma_dot_dot*sin(sigma)+sigma_dot*cos(sigma))]; % 
end

for i=1:aircraft.n_rotor
R_dot_dot_r{i}=[-(sigma_dot_dot*sin(sigma)+sigma_dot*cos(sigma)) 0 (sigma_dot_dot*cos(sigma)-sigma_dot*sin(sigma))
        0          0       0
        -(sigma_dot_dot*cos(sigma)-sigma_dot*sin(sigma)) 0  -(sigma_dot_dot*sin(sigma)+sigma_dot*cos(sigma))]; % 
end

%% --------------------------------------------------------------------------------
% Relative velocities
% ---------------------------------------------------------------------

for i=1:aircraft.n_wing
V_w{i}= R_dot_w{i}*aircraft.r_w_pivot{i};
end
for i=1:aircraft.n_rotor
V_r{i}= R_dot_r{i}*aircraft.r_r_pivot{i};
end

for i=1:aircraft.n_wing
a_w{i}= R_dot_dot_w{i}*aircraft.r_w_pivot{i};
end

for i=1:aircraft.n_rotor
a_r{i}= R_dot_dot_r{i}*aircraft.r_r_pivot{i};
end


%% --------------------------------------------------------------------------------
% Translation Inertia Matrix
% ---------------------------------------------------------------------

for i=1:aircraft.n_wing
R_T_w{i}=[aircraft.r_w{i}(2)^2+aircraft.r_w{i}(3)^2   -aircraft.r_w{i}(1)*aircraft.r_w{i}(2)   -aircraft.r_w{i}(1)*aircraft.r_w{i}(3)
        -aircraft.r_w{i}(1)*aircraft.r_w{i}(2)   aircraft.r_w{i}(1)^2+aircraft.r_w{i}(3)^2   -aircraft.r_w{i}(2)*aircraft.r_w{i}(3)         
         -aircraft.r_w{i}(1)*aircraft.r_w{i}(3)   -aircraft.r_w{i}(2)*aircraft.r_w{i}(3)    aircraft.r_w{i}(1)^2+aircraft.r_w{i}(2)^2];
end

for i=1:aircraft.n_rotor
R_T_r{i}=[aircraft.r_r{i}(2)^2+aircraft.r_r{i}(3)^2   -aircraft.r_r{i}(1)*aircraft.r_r{i}(2)   -aircraft.r_r{i}(1)*aircraft.r_r{i}(3)
        -aircraft.r_r{i}(1)*aircraft.r_r{i}(2)   aircraft.r_r{i}(1)^2+aircraft.r_r{i}(3)^2   -aircraft.r_r{i}(2)*aircraft.r_r{i}(3)         
         -aircraft.r_r{i}(1)*aircraft.r_r{i}(3)   -aircraft.r_r{i}(2)*aircraft.r_r{i}(3)    aircraft.r_r{i}(1)^2+aircraft.r_r{i}(2)^2];
end


for i=1:aircraft.n_wing
R_dot_T_w{i}=[2*(aircraft.r_w{i}(2)*V_w{i}(2)+aircraft.r_w{i}(3)*V_w{i}(3))   -(V_w{i}(1)*aircraft.r_w{i}(2)+aircraft.r_w{i}(1)*V_w{i}(2))   -(V_w{i}(1)*aircraft.r_w{i}(3)+aircraft.r_w{i}(1)*V_w{i}(3))   
        -(V_w{i}(1)*aircraft.r_w{i}(2)+aircraft.r_w{i}(1)*V_w{i}(2))   2*(aircraft.r_w{i}(1)*V_w{i}(1)+aircraft.r_w{i}(3)*V_w{i}(3))   -(V_w{i}(2)*aircraft.r_w{i}(3)+aircraft.r_w{i}(2)*V_w{i}(3))         
        -(V_w{i}(1)*aircraft.r_w{i}(3)+aircraft.r_w{i}(1)*V_w{i}(3))   -(V_w{i}(2)*aircraft.r_w{i}(3)+aircraft.r_w{i}(2)*V_w{i}(3))    2*(aircraft.r_w{i}(1)*V_w{i}(1)+aircraft.r_w{i}(2)*V_w{i}(2))];
end

for i=1:aircraft.n_rotor
R_dot_T_r{i}=[2*(aircraft.r_r{i}(2)*V_r{i}(2)+aircraft.r_r{i}(3)*V_r{i}(3))   -(V_r{i}(1)*aircraft.r_r{i}(2)+aircraft.r_r{i}(1)*V_r{i}(2))   -(V_r{i}(1)*aircraft.r_r{i}(3)+aircraft.r_r{i}(1)*V_r{i}(3))   
        -(V_r{i}(1)*aircraft.r_r{i}(2)+aircraft.r_r{i}(1)*V_r{i}(2))   2*(aircraft.r_r{i}(1)*V_r{i}(1)+aircraft.r_r{i}(3)*V_r{i}(3))   -(V_r{i}(2)*aircraft.r_r{i}(3)+aircraft.r_r{i}(2)*V_r{i}(3))         
        -(V_r{i}(1)*aircraft.r_r{i}(3)+aircraft.r_r{i}(1)*V_r{i}(3))   -(V_r{i}(2)*aircraft.r_r{i}(3)+aircraft.r_r{i}(2)*V_r{i}(3))    2*(aircraft.r_r{i}(1)*V_r{i}(1)+aircraft.r_r{i}(2)*V_r{i}(2))];
end

%% --------------------------------------------------------------------------------
% Rotation Inertia Matrix
% ---------------------------------------------------------------------


for i=1:aircraft.n_wing
T_w{i}=[cos(sigma) 0 0 
        0 1 0 
        0 0 cos(sigma)];

T_dot_w{i}=[-sigma_dot*sin(sigma) 0 0
        0 1 0 
        0 0 -sigma_dot*sin(sigma)];
    
end

for i=1:aircraft.n_rotor
T_r{i}=[cos(sigma) 0 0 
        0 1 0 
        0 0 cos(sigma)];
    
T_dot_r{i}=[-sigma_dot*sin(sigma) 0 0 
        0 1 0 
        0 0 -sigma_dot*sin(sigma)];
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


A=aircraft.I_B;

for i=1:aircraft.n_wing
    A=A+T_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_w{i}' -skew(aircraft.m_w{i}*skew(aircraft.r_w{i})*aircraft.r_w{i}); 
end

for i=1:aircraft.n_rotor
    A=A+T_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_r{i}' -skew(aircraft.m_r{i}*skew(aircraft.r_r{i})*aircraft.r_r{i}); 
end




%% B calculation

B=omega_B*aircraft.I_B;

for i=1:aircraft.n_wing
    B=B + T_dot_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_w{i}' ...
        + T_w{i}*aircraft.m_w{i}*R_dot_T_w{i}*T_w{i}' ...
        + T_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_dot_w{i}' ...
        + omega_B*T_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_w{i}' ...
        - aircraft.m_w{i}*skew(R_dot_w{i}*skew(aircraft.r_w_pivot{i})*aircraft.r_w{i})...
        - aircraft.m_w{i}*skew(skew(aircraft.r_w{i})*omega_B*aircraft.r_w{i});
end

for i=1:aircraft.n_rotor
    B=B + T_dot_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_r{i}' ...
        + T_r{i}*aircraft.m_r{i}*R_dot_T_r{i}*T_r{i}' ...
        + T_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_dot_r{i}' ...
        + omega_B*T_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_r{i}' ...
        - aircraft.m_r{i}*skew(R_dot_r{i}*skew(aircraft.r_r_pivot{i})*aircraft.r_r{i})...
        - aircraft.m_r{i}*skew(skew(aircraft.r_r{i})*omega_B*aircraft.r_r{i});
end


%% C calculation
C= aircraft.m_w{1}*skew(aircraft.r_w{1});

for i=2:aircraft.n_wing
    C= C + aircraft.m_w{i}*skew(aircraft.r_w{i});
end

for i=1:aircraft.n_rotor
    C= C + aircraft.m_r{i}*skew(aircraft.r_r{i});
end

%% D calculation

D= [0;0;0];

for i=1:aircraft.n_wing
    D= D + aircraft.m_w{i}*( skew(R_dot_w{i}*aircraft.r_w{i}) ...
        + skew(aircraft.r_w{i})*omega_B );
end

for i=1:aircraft.n_rotor
    D= D + aircraft.m_r{i}*( skew(R_dot_r{i}*aircraft.r_r_pivot{i}) ...
        + skew(aircraft.r_r{i})*omega_B );
end

%% E calculation

E=[0
    0
    0];

for i=1:aircraft.n_wing
    E=E +( T_dot_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_w{i}' ...
        + T_w{i}*aircraft.m_w{i}*R_dot_T_w{i}*T_w{i}' ...
        + T_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_dot_w{i}' )*R_w{i}*w_w_w{i} ...
        + T_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_dot_w{i}'*(R_dot_w{i}*w_w_w{i} + R_w{i}*w_dot_w_w{i}) ...
        + omega_B*T_w{i}*(aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i})*T_w{i}'*R_w{i}*w_w_w{i}  ...
        + aircraft.m_w{i}*(R_dot_w{i}*skew(aircraft.r_w_pivot{i})*R_dot_w{i} + skew(aircraft.r_w{i})*(2*omega_B*R_dot_w{i}+R_dot_dot_w{i} ))*aircraft.r_w_pivot{i};
end

for i=1:aircraft.n_rotor
    
    if(i<=2)
       j=1;
    elseif(i==3 || i==4)
       j=2;
    elseif(i==5)
       j=3;
    else
       j=4;
    end
    
    E=E +( T_dot_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_r{i}' ...
        + T_r{i}*aircraft.m_r{i}*R_dot_T_r{i}*T_r{i}' ...
        + T_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_dot_r{i}' )*(R_r{i}*w_r_r{i}+R_w{j}*w_w_w{j}) ...
        + T_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_dot_r{i}'*(R_dot_r{i}*w_r_r{i}+R_r{i}*w_dot_r_r{i} +R_dot_w{j}*w_w_w{j}+R_w{j}*w_dot_w_w{j}) ...
        + omega_B*T_r{i}*(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})*T_r{i}'*(R_r{i}*w_r_r{i}+R_w{j}*w_w_w{j})...
        + aircraft.m_r{i}*(R_dot_r{i}*skew(aircraft.r_r_pivot{i})*R_dot_r{i} + skew(aircraft.r_r{i})*(2*omega_B*R_dot_r{i}+R_dot_dot_r{i} ))*aircraft.r_r_pivot{i};
end

%%  Mp calculation

Mp=skew(aircraft.r_b_0)*aircraft.m_B*C_bv*g;
   
for i=1:aircraft.n_wing
    Mp=Mp+ skew(aircraft.r_w{i})*aircraft.m_w{i}*C_bv*g;
end

for i=1:aircraft.n_rotor
    Mp=Mp+ skew(aircraft.r_r{i})*aircraft.m_r{i}*C_bv*g;
end

%% Aerodynamic torques on body
% Torque_B =[0
%            0
%            0];
[~, M_B_aero, Y_aero] = aero_loads(X,U,aircraft);   
[~,M_B_prop, ~] = prop_loads(X,U,aircraft);


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

[F_B_aero ,~, ~] = aero_loads(X,U,aircraft); 
[F_B_prop ,~, ~] = prop_loads(X,U,aircraft);

F_B = F_B_prop+ F_B_aero;

F=[0
   0
   0];


for i=1:aircraft.n_wing
    F = F ...
        + aircraft.m_w{i}*( (skew(w_dot) + skew(w)*skew(w))*aircraft.r_w{i} ...
        + 2*skew(w)*V_w{i} + a_w{i});
end

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
            
             

