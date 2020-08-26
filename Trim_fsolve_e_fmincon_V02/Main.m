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

if isfile('trim_data.mat') == 0

cg_x= 0.40;      % PN=0.3 Catia = 0.3336
cg_z= 0.0922;    % PN=    Catia = 0.0922

aircraft.r_0(1)= cg_x;
aircraft.r_0(3)= cg_z;

V_eq = 30; % m/s
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


x_eq_0 = zeros(13,1);
x_eq_0(4) = V_eq;
x_eq_0(end) = 0.5;
options = optimset('Display','iter','TolFun',1e-10,'TolX',1e-10);
% options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
[x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq_0,options,aircraft,trim_par);
% while exitflag<1
%     [x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq,options,aircraft,trim_par);
% end

[~,X_eq,U_eq,Y_eq] = trim_function(x_eq,aircraft,trim_par);


save trim_data.mat 

else

load trim_data.mat

end

fprintf('----- TRIMMED FLIGHT PARAMETERS -----\n\n');
% fprintf('   %-10s = %10.4f %-4s\n','x_CG',xCG,'m');
% fprintf('   %-10s = %10.4f %-4s\n','y_CG',yCG,'m');
% fprintf('   %-10s = %10.4f %-4s\n','z_CG',zCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','gamma',trim_par.gamma*180/pi,'deg');
fprintf('   %-10s = %10.4f %-4s\n','chi',trim_par.chi*180/pi,'deg');
fprintf('   %-10s = %10.4f %-4s\n','phi_dot',trim_par.phi_dot*180/pi,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','theta_dot',trim_par.theta_dot*180/pi,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','psi_dot',trim_par.psi_dot*180/pi,'deg/s');
fprintf('   %-10s = %10.2f %-4s\n','Sigma',sigma_eq*180/pi,'deg');
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
fprintf('   %-10s = %10.4f %-4s\n','CG',cg_x,'');
fprintf('   %-10s = %10.4f %-4s\n','CL',Y_eq(10),'');



%% --------------------------------------------------------------------------------
% --------------------------------------------------------------------------------
% Simulation
% ---------------------------------------------------------------------
% --------------------------------------------------------------------------------

%% Verifica a condição inicial de Residuos da ODE15i

% Sem controle

% t0= 0;
% y0= X_eq;
% fixed_y0= ones(1,num_states);
% 
% yp0= zeros(1,num_states);
% fixed_yp0 = [];
% 
% [X_0_new,X_dot_0_new] = decic(@dynamics,t0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq,aircraft);
% 
% dt= 0.1;
% tf = 1;
% 
% Res = dynamics(0,X_0_new,X_dot_0_new,U_eq,aircraft);
% 
% if sumabs(Res)>10^-5
%     error("Erro")
% end

%Com controle

t0= 0;
y0= [X_eq
    U_eq];
fixed_y0= ones(1,num_states+4);

yp0= zeros(1,num_states+4);
fixed_yp0 = [];

[X_0_new,X_dot_0_new] = decic(@Hori_flight_15i,t0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq,aircraft,X_eq,Y_eq);

    
dt= 0.05;
tf = 5;

Res = Hori_flight_15i(0,X_0_new,X_dot_0_new,U_eq,aircraft,X_eq,Y_eq);

if sumabs(Res)>10^-5
    error("Erro")
end




%% Simulation
T= 0:dt:tf;

% Sem controle
% [t,X] = ode15i(@dynamics,T,X_0_new,X_dot_0_new,[],U_eq,aircraft);


% Com controle
[t,X] = ode15i(@Hori_flight_15i,T,X_0_new,X_dot_0_new,[],U_eq,aircraft,X_eq,Y_eq);



%% Y calculation

Y= zeros (length (T), length(Y_eq)+4); 

for i_t=1:length(T)
    
    t0= t(i_t);
    y0= X(i_t,:);
    fixed_y0= ones(1,num_states+4);

    yp0= zeros(1,num_states+4);
    fixed_yp0 = [];
    
    [y0_new,yp0_new] = decic(@Hori_flight_15i,t0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq,aircraft,X_eq,Y_eq);
    
    [~,Yt] = Hori_flight_15i(t(i_t),y0_new,yp0_new,U_eq,aircraft,X_eq,Y_eq);
    Y(i_t,:)= Yt.';
end


%% Linearization

A = zeros(length (X_eq),length (X_eq));
B = zeros(length (X_eq),length (U_eq));
C = zeros(length (Y_eq),length (X_eq));
D = zeros(length (Y_eq),length (U_eq));

h=1e-5;

for i_X=1:length(X_eq)
    dX = zeros(length(X_eq),1);
    dX(i_X)= h;
    
    y0= X_eq + dX;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_plus ,Xdot_plus] = decic(@dynamics,t0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq,aircraft);
    [~, Y_plus]= dynamics(0,X_plus,Xdot_plus,U_eq,aircraft);
    
       
    y0= X_eq - dX;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_minus ,Xdot_minus] = decic(@dynamics,t0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq,aircraft);
    [~, Y_minus]= dynamics(0,X_minus,Xdot_minus,U_eq,aircraft);
    
    A(:,i_X) = (Xdot_plus-Xdot_minus)/(2*dX(i_X));
    C(:,i_X) = (Y_plus-Y_minus)/(2*dX(i_X));
end


for i_U=1:length(U_eq)
    dU = zeros(length(U_eq),1);
    dU(i_U)= h;
    
     y0= X_eq;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_plus ,Xdot_plus] = decic(@dynamics,t0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq+dU,aircraft);
    [~, Y_plus]= dynamics(0,X_plus,Xdot_plus,U_eq+dU,aircraft);
    
    y0= X_eq;
    fixed_y0= ones(1,num_states);

    yp0= zeros(1,num_states);
    fixed_yp0 = [];

    [X_minus ,Xdot_minus] = decic(@dynamics,t0,y0,fixed_y0,yp0,fixed_yp0,[],U_eq-dU,aircraft);
    [~, Y_minus]= dynamics(0,X_minus,Xdot_minus,U_eq-dU,aircraft);

    B(:,i_U) = (Xdot_plus-Xdot_minus)/(2*dU(i_U));
    D(:,i_U) = (Y_plus-Y_minus)/(2*dU(i_U));
end


save ABCD.mat A B C D

%% Static Margin

i_alpha=8;
i_CL =10;
i_Cm= 11;
i_de=2;

Calpha = C(i_alpha,:);
CCL = C(i_CL,:);
CCm= C(i_Cm,:);

Cm_alpha = CCm(6)/Calpha(6);
CL_alpha = CCL(6)/Calpha(6);

Kn =-Cm_alpha/CL_alpha;

DCL =D(i_CL,:);
DCm= D(i_Cm,:);

Cm_delta_e = DCm(i_de);
CL_delta_e = DCL(i_de);

%% --------------------------------------------------------------------------------
% --------------------------------------------------------------------------------
% Results
% ---------------------------------------------------------------------
% --------------------------------------------------------------------------------

visualizator_VTOL(X)

%%
figure
plot(T,-X(:,3))
xlabel('T')
ylabel('h (-z)')


%%
figure
subplot(3,1,1)
plot(T,X(:,10)*180/pi)
xlabel('T')
ylabel('p (deg/s)')

subplot(3,1,2)
plot(T,X(:,11)*180/pi)
xlabel('T')
ylabel('q (deg/s)')

subplot(3,1,3)
plot(T,X(:,12)*180/pi)
xlabel('T')
ylabel('r (deg/s)')

%%
figure
subplot(3,1,1)
plot(T,X(:,4))
xlabel('T')
ylabel('Vx')

subplot(3,1,2)
plot(T,X(:,5))
xlabel('T')
ylabel('Vy')

subplot(3,1,3)
plot(T,X(:,6))
xlabel('T')
ylabel('Vz')

%%
figure
subplot(3,1,1)
plot(T,Y(:,1))
xlabel('T')
ylabel('Force x')

subplot(3,1,2)
plot(T,Y(:,2))
xlabel('T')
ylabel('Force y')

subplot(3,1,3)
plot(T,Y(:,3))
xlabel('T')
ylabel('Force z')

%%
figure
subplot(3,1,1)
plot(T,Y(:,4))
xlabel('T')
ylabel('Torque x')

subplot(3,1,2)
plot(T,Y(:,5))
xlabel('T')
ylabel('Torque y')

subplot(3,1,3)
plot(T,Y(:,6))
xlabel('T')
ylabel('Torque z')

%%
figure
subplot(2,1,1)
plot(T,X(:,13)*180/pi)
xlabel('T')
ylabel('Sigma')

subplot(2,1,2)
plot(T,X(:,14)*180/pi)
xlabel('T')
ylabel('Sigma_dot')

%%

if length(X(1,:))==18
    
    figure
    subplot(3,1,1)
    plot(T,X(:,15),T,Y(:,end-3))
    xlabel('T')
    ylabel('Thrust')
    legend ('Real','Comandado')
    
    
    subplot(3,1,2)
    plot(T,X(:,16),T,Y(:,end-2))
    xlabel('T')
    ylabel('Elevator')
    legend ('Real','Comandado')

    subplot(3,1,3)
    plot(T,X(:,17),T,Y(:,end-1))
    xlabel('T')
    ylabel('Aileron')
    legend ('Real','Comandado')

end
    
%%
figure
subplot(2,1,1)
plot(T,Y(:,8))
xlabel('T')
ylabel('Alpha')

subplot(2,1,2)
plot(T,X(:,9))
xlabel('T')
ylabel('Beta')

%%

figure
subplot(3,1,1)
plot(T,X(:,7)*180/pi)
xlabel('T')
ylabel('Phi')

subplot(3,1,2)
plot(T,X(:,8)*180/pi)
xlabel('T')
ylabel('Theta')

subplot(3,1,3)
plot(T,X(:,9)*180/pi)
xlabel('T')
ylabel('Psy')
