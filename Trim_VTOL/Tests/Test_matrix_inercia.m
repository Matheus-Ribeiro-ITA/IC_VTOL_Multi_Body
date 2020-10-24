clear
clc

addpath Sub_functions

load Aircraft_data/aircraft



sigma = 90*pi/180;



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


%% --------------------------------------------------------------------------------
% Rotation Inertia Matrix
% ---------------------------------------------------------------------


for i=1:aircraft.n_wing
T_w{i}=[cos(sigma) 0 sin(sigma) 
        0 1 0 
        -sin(sigma) 0 cos(sigma)];
    
end

for i=1:aircraft.n_rotor
T_r{i}=[cos(sigma) 0 sin(sigma)  
        0 1 0 
        -sin(sigma)  0 cos(sigma)];
end


A=aircraft.I_B;

% for i=1:aircraft.n_wing
%     A=A+aircraft.I_w{i} + aircraft.m_w{i}*R_T_w{i};
% end

for i=1:aircraft.n_rotor
    A=A+(aircraft.I_r{i} + aircraft.m_r{i}*R_T_r{i})';
end

A

% test = T_r{1}*aircraft.I_r{1}*T_r{1}' + aircraft.m_r{1}*R_T_r{1}
% test02 = T_r{1}*(aircraft.I_r{1} + aircraft.m_r{1}*R_T_r{1})*T_r{1}'





test03=aircraft.I_B;

for i=1:aircraft.n_rotor
r_r_pivot{i}=DCM_original(2,-sigma)*aircraft.r_r_pivot{i}; % Gira o pivot
aircraft.r_r{i} = r_r_pivot{i} + aircraft.r_pivot{i}; % Muda pra ref. body
end

for i=1:aircraft.n_rotor
R_T_r{i}=[aircraft.r_r{i}(2)^2+aircraft.r_r{i}(3)^2   -aircraft.r_r{i}(1)*aircraft.r_r{i}(2)   -aircraft.r_r{i}(1)*aircraft.r_r{i}(3)
        -aircraft.r_r{i}(1)*aircraft.r_r{i}(2)   aircraft.r_r{i}(1)^2+aircraft.r_r{i}(3)^2   -aircraft.r_r{i}(2)*aircraft.r_r{i}(3)         
         -aircraft.r_r{i}(1)*aircraft.r_r{i}(3)   -aircraft.r_r{i}(2)*aircraft.r_r{i}(3)    aircraft.r_r{i}(1)^2+aircraft.r_r{i}(2)^2];
end


for i=1:aircraft.n_rotor
test03 = test03+ T_r{i}*aircraft.I_r{i}*T_r{i}' + aircraft.m_r{i}*R_T_r{i};
end

d_cg= -[-28*10^(-3); 0 ; +24*10^(-3)]

R_T_d_cg=[d_cg(2)^2+d_cg(3)^2   -d_cg(1)*d_cg(2)   -d_cg(1)*d_cg(3)
        -d_cg(1)*d_cg(2)   d_cg(1)^2+d_cg(3)^2   -d_cg(2)*d_cg(3)         
         -d_cg(1)*d_cg(3)   -d_cg(2)*d_cg(3)    d_cg(1)^2+d_cg(2)^2];


test03 = test03 + aircraft.m_total*R_T_d_cg

a=DCM_original(2,-sigma)*[1;0;0]
