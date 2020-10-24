%% eVTOL Dynamics Modeling
% Author: Matheus Ribeiro T-21

clear
clc

tic
%% --------------------------------------------------------------------------------
% Aircraft Parameters and Functions
% ---------------------------------------------------------------------
addpath ..\
addpath ..\Sub_functions
addpath ..\Trim_results_multi_body
addpath ..\Trimagem_multi_body
addpath ..\Aircraft_data

load aircraft
% load trim_results_opt_v11.mat
%% --------------------------------------------------------------------------------
%  --------------------------------------------------------------------------------
%  Initial conditions
%  ---------------------------------------------------------------------
% --------------------------------------------------------------------------------


% Gravity
global g
g=[0;0;9.8];

% Position

x=0;
y=0;
h=0;


% Rate of tilt
sigma=0*pi/180; % Wing tilt angle (0=bottom) [All change together]
sigma_dot= -10*pi/180;
sigma_dot_dot= -5*pi/180;

% Aircraft atitude
euler=[0
       0
       0]; % Phi;Theta;Psi (always in rad)
   
% Angular velocities
w=[0
   0
   0*pi/180]; % p;q;r (always in rad)

w_dot=[0
       0
       0];
% Velocity

V_dot=[0
       0
       0];
   
V=    [0
       0
       0];

   

%% State Variables
X_0=[x;y;h      %(1:3)
    V           %(4:6)
    euler       %(7:9)
    w           %(10:12)
    sigma       %(13)
    sigma_dot]; %(14)     

X_dot_0= [V
       V_dot
       w
       w_dot
       sigma_dot
       sigma_dot_dot];

 U_0 = [0.2 % Motor
     0  % Elevator
     0  % Aileron
     0];% Rudder
 
num_states= length(X_0);
%%

Y_0 = zeros(11,1);



%% Trim calculation (Ainda nao funciona)

i=1;

% First initial conditions

cg(1,1)= 0.29;
cg(2,1) = 0;        % PN=0.3 Catia = 0.3336
cg(3,1)= 0.0922;    % PN=    Catia = 0.0922

carga = 1;

% Conserta o CG do corpo rigido
aircraft = aircraft_gen(carga,cg);
aircraft.r_0


aircraft.spin_speed_RPM = 1000;
% V_eq = 2; % m/s
h_eq = 0;
chi_eq = 0;
gamma_eq = 0;
phi_dot_eq = 0;
theta_dot_eq = 0;
psi_dot_eq = 0;
sigma_eq = 0;


beta_eq = 0;


for V_eq = 10.5:0.1:12

trim_par = struct('V',V_eq,...
    'h',h_eq,...
    'chi',chi_eq,...
    'gamma',gamma_eq,...
    'phi_dot',phi_dot_eq,...
    'theta_dot',theta_dot_eq,...
    'psi_dot',psi_dot_eq, ...
    'beta',beta_eq, ...
    'sigma',sigma_eq, ...
    'sigma_dot',sigma_dot);

    
% Ultima solucao

ini_0= struct('X_0',X_0,'U_0',U_0,'Y_0',Y_0);

options = optimset('Display','iter','TolFun',1e-10,'TolX',1e-5);
fun= @(alpha_eq)( trim_function_fminse(alpha_eq,V_eq,aircraft,trim_par,ini_0));
A=[];
b=[];
Aeq=[];
beq=[];
lb=0*pi/180;
ub=+16*pi/180;
nonlcon = [];

alpha_eq = fmincon(fun,8*pi/180,A,b,Aeq,beq,lb,ub,nonlcon,options);


trim_par.alpha_eq = alpha_eq;

[~,X_eq,U_eq,Y_eq] = trim_function_fminse(alpha_eq,V_eq,aircraft,trim_par,ini_0);


fprintf('----- TRIMMED FLIGHT PARAMETERS -----\n\n');
% fprintf('   %-10s = %10.4f %-4s\n','x_CG',xCG,'m');
% fprintf('   %-10s = %10.4f %-4s\n','y_CG',yCG,'m');
% fprintf('   %-10s = %10.4f %-4s\n','z_CG',zCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','gamma',trim_par.gamma*180/pi,'deg');
fprintf('   %-10s = %10.4f %-4s\n','chi',trim_par.chi*180/pi,'deg');
fprintf('   %-10s = %10.4f %-4s\n','phi_dot',trim_par.phi_dot*180/pi,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','theta_dot',trim_par.theta_dot*180/pi,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','psi_dot',trim_par.psi_dot*180/pi,'deg/s');
fprintf('   %-10s = %10.2f %-4s\n','Sigma',X_eq(13,1)*180/pi,'deg');
fprintf('\n');
fprintf('   %-10s = %10.4f %-4s\n','alpha',Y_eq(8),'deg');
fprintf('   %-10s = %10.4f %-4s\n','beta',Y_eq(9),'deg');
fprintf('   %-10s = %10.4f %-4s\n','q',X_eq(11)*180/pi,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','theta',X_eq(8)*180/pi,'deg');
fprintf('   %-10s = %10.1f %-4s\n','H',X_eq(3),'m');
% fprintf('   %-10s = %10.4f %-4s\n','beta',Y_eq(3),'deg');
fprintf('   %-10s = %10.4f %-4s\n','phi',X_eq(7)*180/pi,'deg');
fprintf('   %-10s = %10.4f %-4s\n','p',X_eq(10)*180/pi,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','r',X_eq(12)*180/pi,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','psi',X_eq(9)*180/pi,'deg');
fprintf('\n');
% fprintf('   %-10s = %10.2f %-4s\n','nx_C_b',Y_eq(end-5),'%');
% fprintf('   %-10s = %10.2f %-4s\n','ny_C_b',Y_eq(end-4),'%');
% fprintf('   %-10s = %10.2f %-4s\n','nz_C_b',Y_eq(end-3),'%');
% fprintf('\n');
% fprintf('   %-10s = %10.2f %-4s\n','nx_P_b',Y_eq(end-5),'%');
% fprintf('   %-10s = %10.2f %-4s\n','ny_P_b',Y_eq(end-4),'%');
% fprintf('   %-10s = %10.2f %-4s\n','nz_P_b',Y_eq(end-3),'%');
fprintf('   %-10s = %10.2f %-4s\n','throttle',100*U_eq(1),'%');
fprintf('   %-10s = %10.4f %-4s\n','delta_e',U_eq(2),'deg');
fprintf('   %-10s = %10.4f %-4s\n','delta_a',U_eq(3),'deg');
fprintf('   %-10s = %10.4f %-4s\n','delta_r',U_eq(4),'deg');

fprintf('\n');
fprintf('   %-10s = %10.4f %-4s\n','V',sqrt(X_eq(4,1)^2+X_eq(5,1)^2+X_eq(6,1)^2),' m/s');
fprintf('   %-10s = %10.4f %-4s\n','V_b_x',X_eq(4,1),' m/s');
fprintf('   %-10s = %10.4f %-4s\n','V_b_y',X_eq(5,1),' m/s');
fprintf('   %-10s = %10.4f %-4s\n','CG',cg(1,1),'');
fprintf('   %-10s = %10.4f %-4s\n','CL',Y_eq(10),'');
% end

Mat_X_eq(:,i)=X_eq;
Mat_U_eq(:,i)=U_eq;
Mat_Y_eq(:,i)=Y_eq;
Mat_V_eq(i)= V_eq;


% Usar ultima solução

X_0 = X_eq;
Y_0 = Y_eq;
U_0 = U_eq;

i=i+1;




% end

end
%%
save ..\Trim_results_Rudder/trim_results_opt_5.mat Mat_X_eq Mat_U_eq Mat_Y_eq Mat_V_eq
time_opt= toc

save time.mat time_opt