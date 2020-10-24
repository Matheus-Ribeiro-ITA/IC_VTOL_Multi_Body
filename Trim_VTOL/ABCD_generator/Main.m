%% ABCD Generator (Incomplete)
% Author: Matheus Ribeiro T-21
% gmail: matehus.ribeiro.aer@gmail.com
% Version: 0.0.1 
% Description: When complete will generate states
%              matrixs A, B, C, D and E.


clear
clc

%% --------------------------------------------------------------------------------
% Paths
% ---------------------------------------------------------------------

addpath ..\

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



%% Trim load

if isfile('trim_results_opt_08.mat') == 1
    
    cg_x= 0.29;      % PN=0.3 Catia = 0.3336
    cg_z= 0.0922;    % PN=    Catia = 0.0922
    
    aircraft.r_0(1)= cg_x;
    aircraft.r_0(3)= cg_z;
    
    V_eq=11.8;
    h_eq = 0;
    chi_eq = 0;
    gamma_eq = 0;
    phi_dot_eq = 0;
    theta_dot_eq = 0;
    psi_dot_eq = 0;
    sigma_eq = 0*pi/180;
    
    
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
    
    
    load trim_results_opt_08.mat
end

% ABCD Calculation

n = length(Mat_V_eq);
   

% A = zeros(length (X_eq),length (X_eq));
% B = zeros(length (X_eq),length (U_eq));
% C = zeros(length (Y_eq),length (X_eq));
% D = zeros(length (Y_eq),length (U_eq));


for i=1:n
    
    X_eq=Mat_X_eq(:,i);
    U_eq=Mat_U_eq(:,i);
    Y_eq=Mat_Y_eq(:,i);

    

%% Linearization



    h=1e-5;

for i_X=1:length(X_eq)
    
    dX = zeros(length(X_eq),1);
    dX(i_X)= h;
    
    y0= X_eq + dX;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_plus ,Xdot_plus] = decic(@dynamics,0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq,aircraft);
    [~, Y_plus]= dynamics(0,X_plus,Xdot_plus,U_eq,aircraft);
    
       
    y0= X_eq - dX;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_minus ,Xdot_minus] = decic(@dynamics,0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq,aircraft);
    [~, Y_minus]= dynamics(0,X_minus,Xdot_minus,U_eq,aircraft);
    
    A{i}(:,i_X) = (Xdot_plus-Xdot_minus)/(2*dX(i_X));
    C{i}(:,i_X) = (Y_plus-Y_minus)/(2*dX(i_X));
end


for i_U=1:length(U_eq)
    dU = zeros(length(U_eq),1);
    dU(i_U)= h;
    
     y0= X_eq;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_plus ,Xdot_plus] = decic(@dynamics,0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq+dU,aircraft);
    [~, Y_plus]= dynamics(0,X_plus,Xdot_plus,U_eq+dU,aircraft);
    
    y0= X_eq;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_minus ,Xdot_minus] = decic(@dynamics,0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq-dU,aircraft);
    [~, Y_minus]= dynamics(0,X_minus,Xdot_minus,U_eq-dU,aircraft);

    B{i}(:,i_U) = (Xdot_plus-Xdot_minus)/(2*dU(i_U));
    D{i}(:,i_U) = (Y_plus-Y_minus)/(2*dU(i_U));
end

end
save ABCD.mat A B C D

