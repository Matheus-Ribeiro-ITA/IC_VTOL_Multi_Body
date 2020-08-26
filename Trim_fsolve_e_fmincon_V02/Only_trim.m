%% eVTOL Dynamics Modeling
% Author: Matheus Ribeiro T-21

clear
clc

%% --------------------------------------------------------------------------------
% Aircraft Parameters
% ---------------------------------------------------------------------

load aircraft

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
sigma_dot=0;
sigma_dot_dot=0;

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

 U_0 = [0.5 % Motor
     0  % Elevator
     0  % Aileron
     0];% Rudder
 
num_states= length(X_0);
%%

Y_eq = zeros(11,1);



%% Trim calculation (Ainda nao funciona)


% for alpha = 0.5:0.5:16

V_eq = 12.2; % m/s
alpha_eq = 11.82*pi/180;


cg_x= 0.29;     % PN=0.3 Catia = 0.3336
cg_z= 0.0922;    % PN=    Catia = 0.0922

aircraft.r_0(1)= cg_x;
aircraft.r_0(3)= cg_z;


h_eq = 0;
chi_eq = 0;
gamma_eq = 0;
phi_dot_eq = 0;
theta_dot_eq = 0;
psi_dot_eq = 0;
sigma_eq = 0;


beta_eq = 0;

trim_par = struct('V',V_eq,...
    'h',h_eq,...
    'chi',chi_eq,...
    'gamma',gamma_eq,...
    'phi_dot',phi_dot_eq,...
    'theta_dot',theta_dot_eq,...
    'psi_dot',psi_dot_eq, ...
    'beta',beta_eq, ...
    'sigma',sigma_eq, ...
    'alpha_eq',alpha_eq);


x_eq_0 = zeros(5,1);
x_eq_0(2) = 80*pi/180;       % Sigma
x_eq_0(3) = V_eq;            % u
x_eq_0(5) = 1;               % Throttle
options = optimset('Display','iter','TolFun',1e-10,'TolX',1e-10);
% options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
[x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function_sigma_alpha,x_eq_0,options,aircraft,trim_par);
while exitflag<1
    [x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function_sigma_alpha,x_eq,options,aircraft,trim_par);
end

[~,X_eq,U_eq,Y_eq] = trim_function_sigma_alpha(x_eq,aircraft,trim_par);


% Trigonometry correction

% if X_eq(13,1)*180/pi>360
%     X_eq(13,1)=X_eq(13,1)-2*pi;
% end
% 
% if X_eq(13,1)*180/pi<270 && X_eq(13,1)*180/pi>180 && U_eq(1)<0
%     U_eq(1)=-U_eq(1);
%     X_eq(13,1)=X_eq(13,1)-pi;
% end



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
fprintf('   %-10s = %10.4f %-4s\n','CG',cg_x,'');
fprintf('   %-10s = %10.4f %-4s\n','CL',Y_eq(10),'');
% end