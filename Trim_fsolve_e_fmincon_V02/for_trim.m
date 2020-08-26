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

i=1;
for sigma_var=0:5:85

    
cg_x= 0.35;      % PN=0.3 Catia = 0.3336, c= 0.25
cg_z= 0.0922;    % PN=    Catia = 0.0922

aircraft.r_0(1)= cg_x;
aircraft.r_0(3)= cg_z;

V_eq = 15; % m/s
h_eq = 0;
chi_eq = 0;
gamma_eq = 0;
phi_dot_eq = 0;
theta_dot_eq = 0;
psi_dot_eq = 0;
sigma_eq = sigma_var*pi/180;


beta_eq = 0;

trim_par = struct('V',V_eq,...
    'h',h_eq,...
    'chi',chi_eq,...
    'gamma',gamma_eq,...
    'phi_dot',phi_dot_eq,...
    'theta_dot',theta_dot_eq,...
    'psi_dot',psi_dot_eq, ...
    'beta',beta_eq, ...
    'sigma',sigma_eq);


x_eq_0 = zeros(13,1);
x_eq_0(4) = V_eq;
x_eq_0(end) = 0.5;
options = optimset('Display','iter','TolFun',1e-10,'TolX',1e-10);
% options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
[x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq_0,options,aircraft,trim_par);
while exitflag<1
    [x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq,options,aircraft,trim_par);
end

[~,X_eq,U_eq,Y_eq] = trim_function(x_eq,aircraft,trim_par);


Mat_X_eq(:,i)=X_eq;
Mat_U_eq(:,i)=U_eq;
Mat_Y_eq(:,i)=Y_eq;

i=i+1;
end

%% 


save trim_results_03.mat Mat_X_eq Mat_U_eq Mat_Y_eq V_eq cg_x


