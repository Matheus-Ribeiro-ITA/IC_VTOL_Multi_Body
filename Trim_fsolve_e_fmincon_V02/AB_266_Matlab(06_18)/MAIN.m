clear all
close all
clc

global g
global m2ft ft2m kg2lb lb2kg kg2slug slug2kg deg2rad rad2deg lbf2N N2lbf

g = 9.80665;

m2ft = 1/0.3048;
ft2m = 1/m2ft;
g_ft_s2 = g*m2ft;

lb2kg = 0.45359237;
kg2lb = 1/lb2kg;

% Definition of slug: 1 slug is the mass that is accelerated by 1 ft/s²
% when a force of 1 pound (1 lbf) is exerted on it
slug2kg = g_ft_s2*lb2kg;
kg2slug = 1/slug2kg;

deg2rad = pi/180;
rad2deg = 1/deg2rad;

lbf2N = slug2kg*ft2m;
N2lbf = 1/lbf2N;

%--------------------------------------------------------------------------
% F-16 geometric data:
b = 30*ft2m;                 % Wing span
S = 300*ft2m^2;              % Wing planform area
c = 11.32*ft2m;              % Wing mean aerodynamic chord (m.a.c.)
r_pilot_b = [15; 0; 0]*ft2m; % Pilot position vector, in body system
aircraft.geom = struct('b',b,'c',c,'S',S,...
    'r_pilot_b',r_pilot_b);

%--------------------------------------------------------------------------
% F-16 mass and inertia data:
W = 20500; % lbf
% Mass [slug]:
m = W/g_ft_s2;
% Mass [kg]:
m = m*slug2kg;
% Moments of inertia:
Ixx = 9496*slug2kg*ft2m^2;
Iyy = 55814*slug2kg*ft2m^2;
Izz = 63100*slug2kg*ft2m^2;
% Products of inertia:
Ixy = 0*slug2kg*ft2m^2;
Ixz = 982*slug2kg*ft2m^2;
Iyz = 0*slug2kg*ft2m^2;
J_O_b = [Ixx -Ixy -Ixz
    -Ixy Iyy -Iyz
    -Ixz -Iyz Izz];
% Nominal CG: 0.35c
xCG = 0;
yCG = 0;
zCG = 0;
r_C_b = [xCG yCG zCG].';
aircraft.mass = struct('m',m,'J_O_b',J_O_b,'r_C_b',r_C_b);

%--------------------------------------------------------------------------
% F-16 engine angular momentum:
hex = 160; % slug·ft²/s
hex = hex*slug2kg*ft2m^2;
aircraft.prop = struct('hex',hex);


%--------------------------------------------------------------------------
% Equilibrium calculation:

% aircraft.mass.r_C_b= [0.1*c 0 0].';

V_eq = 502*ft2m;
h_eq = 0;
chi_deg_eq = 0;
gamma_deg_eq = 0;
phi_dot_deg_s_eq = 0.*rad2deg;
theta_dot_deg_s_eq = 0.*rad2deg;
% psi_dot_deg_s_eq = 0.3*rad2deg; %Com curva
psi_dot_deg_s_eq = 0.*rad2deg;

coord_flag='nyp';
beta_deg_eq = 0;
nyp_eq = 0;

trim_par = struct('V',V_eq,...
    'h',h_eq,...
    'chi_deg',chi_deg_eq,...
    'gamma_deg',gamma_deg_eq,...
    'phi_dot_deg_s',phi_dot_deg_s_eq,...
    'theta_dot_deg_s',theta_dot_deg_s_eq,...
    'psi_dot_deg_s',psi_dot_deg_s_eq, ...
    'coord_flag',coord_flag, ...
    'beta_deg',beta_deg_eq, ...
    'nyp',nyp_eq);

x_eq_0 = zeros(14,1);
x_eq_0(1) = V_eq;
options = optimset('Display','iter','TolFun',1e-10,'TolX',1e-10);
[x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq_0,options,aircraft,trim_par);
while exitflag<1
    [x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq,options,aircraft,trim_par);
end
[~,X_eq,U_eq] = trim_function(x_eq,aircraft,trim_par);

[Xdot_eq,Y_eq] = dynamics(0,X_eq,U_eq,aircraft);

fprintf('----- TRIMMED FLIGHT PARAMETERS -----\n\n');
fprintf('   %-10s = %10.4f %-4s\n','x_CG',xCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','y_CG',yCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','z_CG',zCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','gamma',trim_par.gamma_deg,'deg');
fprintf('   %-10s = %10.4f %-4s\n','chi',trim_par.chi_deg,'deg');
fprintf('   %-10s = %10.4f %-4s\n','phi_dot',trim_par.phi_dot_deg_s,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','theta_dot',trim_par.theta_dot_deg_s,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','psi_dot',trim_par.psi_dot_deg_s,'deg/s');
fprintf('\n');
fprintf('   %-10s = %10.2f %-4s\n','V',Y_eq(1),'m/s');
fprintf('   %-10s = %10.4f %-4s\n','alpha',Y_eq(2),'deg');
fprintf('   %-10s = %10.4f %-4s\n','q',X_eq(3),'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','theta',X_eq(4),'deg');
fprintf('   %-10s = %10.1f %-4s\n','H',X_eq(5),'m');
fprintf('   %-10s = %10.4f %-4s\n','beta',Y_eq(3),'deg');
fprintf('   %-10s = %10.4f %-4s\n','phi',X_eq(8),'deg');
fprintf('   %-10s = %10.4f %-4s\n','p',X_eq(9),'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','r',X_eq(10),'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','psi',X_eq(11),'deg');
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
save trim_data.mat X_eq U_eq Y_eq Xdot_eq

% Flight simulation of equilibrium condition
dt= 0.100;
% tf= 360/psi_dot_deg_s_eq;
tf = 50;

T= 0:dt:tf;
X0 =X_eq;
X = ode4(@dynamics,T,X0,U_eq,aircraft);
Y= zeros (length (T), length(Y_eq)); 

for i_t=1:length(T)
    [~,Yt] = dynamics(T(i_t),X(i_t,:).',U_eq,aircraft);
    Y(i_t,:)= Yt.';
end

% figure
% plot (T,Y(:,end))
% xlabel('t [s]')
% ylabel('n_{P,z}')
% 
% figure
% plot3(X(:,12),X(:,6),X(:,5))
% xlabel ('y [m]')
% ylabel ('x [m]')
% zlabel ('h [m]')
% axis equal

% ------------------------------------------------------------------------------------
% Linearization

A = zeros(length (X_eq),length (X_eq));
B = zeros(length (X_eq),length (U_eq));
C = zeros(length (Y_eq),length (X_eq));
D = zeros(length (Y_eq),length (U_eq));

h=1e-5;

for i_X=1:length(X_eq)
    dX = zeros(length(X_eq),1);
    dX(i_X)= h;
    [Xdot_plus, Y_plus]= dynamics(0,X_eq+dX,U_eq,aircraft);
    [Xdot_minus, Y_minus]= dynamics(0,X_eq-dX,U_eq,aircraft);
    A(:,i_X) = (Xdot_plus-Xdot_minus)/(2*dX(i_X));
    C(:,i_X) = (Y_plus-Y_minus)/(2*dX(i_X));
end


for i_U=1:length(U_eq)
    dU = zeros(length(U_eq),1);
    dU(i_U)= h;
    [Xdot_plus, Y_plus]= dynamics(0,X_eq,U_eq+dU,aircraft);
    [Xdot_minus, Y_minus]= dynamics(0,X_eq,U_eq-dU,aircraft);
    B(:,i_U) = (Xdot_plus-Xdot_minus)/(2*dU(i_U));
    D(:,i_U) = (Y_plus-Y_minus)/(2*dU(i_U));
end
%% --------------------------------------------------------------
% Static Margin

i_alpha=2;
i_CL =12;
i_Cm= 8;
i_de=2;

Calpha = C(i_alpha,:);
CCL = C(i_CL,:);
CCm= C(i_Cm,:);

Cm_alpha = CCm(2)/Calpha(2);
CL_alpha = CCL(2)/Calpha(2);

Kn =-Cm_alpha/CL_alpha;

DCL =D(i_CL,:);
DCm= D(i_Cm,:);

Cm_delta_e = DCm(i_de);
CL_delta_e = DCL(i_de);

 %% -------------------------------------------------------------------------------------------
% Equilibrium calculation with wind:

% aircraft.mass.r_C_b= [0.1*c 0 0].';

V_eq = 502*ft2m;
h_eq = 0;
chi_deg_eq = 0;
gamma_deg_eq = 0;
phi_dot_deg_s_eq = 0.*rad2deg;
theta_dot_deg_s_eq = 0.*rad2deg;
% psi_dot_deg_s_eq = 0.3*rad2deg; %Com curva
psi_dot_deg_s_eq = 0.*rad2deg;

Vw_v_eq= [0;0;0]; % Velocidade do vento Vertical local

coord_flag='beta';
beta_deg_eq = 0;
nyp_eq = 0;

trim_par = struct('V',V_eq,...
    'h',h_eq,...
    'chi_deg',chi_deg_eq,...
    'gamma_deg',gamma_deg_eq,...
    'phi_dot_deg_s',phi_dot_deg_s_eq,...
    'theta_dot_deg_s',theta_dot_deg_s_eq,...
    'psi_dot_deg_s',psi_dot_deg_s_eq, ...
    'coord_flag',coord_flag, ...
    'beta_deg',beta_deg_eq, ...
    'nyp',nyp_eq, ...
    'Vw_v',Vw_v_eq );

x_eq_0 = zeros(14,1);
x_eq_0(1) = V_eq;
options = optimset('Display','iter','TolFun',1e-10,'TolX',1e-10);
[x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq_0,options,aircraft,trim_par);
while exitflag<1
    [x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_function,x_eq,options,aircraft,trim_par);
end
[~,X_eq,U_eq] = trim_function(x_eq,aircraft,trim_par);

[Xdot_eq,Y_eq] = dynamics(0,X_eq,U_eq,aircraft,trim_par.Vw_v);

fprintf('----- TRIMMED FLIGHT PARAMETERS -----\n\n');
fprintf('   %-10s = %10.4f %-4s\n','x_CG',xCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','y_CG',yCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','z_CG',zCG,'m');
fprintf('   %-10s = %10.4f %-4s\n','gamma',trim_par.gamma_deg,'deg');
fprintf('   %-10s = %10.4f %-4s\n','chi',trim_par.chi_deg,'deg');
fprintf('   %-10s = %10.4f %-4s\n','phi_dot',trim_par.phi_dot_deg_s,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','theta_dot',trim_par.theta_dot_deg_s,'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','psi_dot',trim_par.psi_dot_deg_s,'deg/s');
fprintf('\n');
fprintf('   %-10s = %10.2f %-4s\n','V',Y_eq(1),'m/s');
fprintf('   %-10s = %10.4f %-4s\n','alpha',Y_eq(2),'deg');
fprintf('   %-10s = %10.4f %-4s\n','q',X_eq(3),'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','theta',X_eq(4),'deg');
fprintf('   %-10s = %10.1f %-4s\n','H',X_eq(5),'m');
fprintf('   %-10s = %10.4f %-4s\n','beta',Y_eq(3),'deg');
fprintf('   %-10s = %10.4f %-4s\n','phi',X_eq(8),'deg');
fprintf('   %-10s = %10.4f %-4s\n','p',X_eq(9),'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','r',X_eq(10),'deg/s');
fprintf('   %-10s = %10.4f %-4s\n','psi',X_eq(11),'deg');
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
% save trim_data.mat X_eq U_eq Y_eq Xdot_eq
% 
% % Flight simulation of equilibrium condition
% dt= 0.100;
% % tf= 360/psi_dot_deg_s_eq;
% tf = 200;
% 
% T= 0:dt:tf;
% X0 =X_eq;
% X = ode4(@dynamics,T,X0,U_eq,aircraft,trim_par.Vw_v);
% Y= zeros (length (T), length(Y_eq)); 
% 
% for i_t=1:length(T)
%     [~,Yt] = dynamics(T(i_t),X(i_t,:).',U_eq,aircraft,trim_par.Vw_v);
%     Y(i_t,:)= Yt.';
% end

% figure
% plot (T,Y(:,end))
% xlabel('t [s]')
% ylabel('n_{P,z}')
% 
% figure
% plot3(X(:,12),X(:,6),X(:,5))
% xlabel ('y [m]')
% ylabel ('x [m]')
% zlabel ('h [m]')
% axis equal

%---------------------------------------------------------------------------------------
% Flight simulation of Pitch SAS in one minus-cosine
dt= 0.010;
% tf= 360/psi_dot_deg_s_eq;
tf = 20;

T= 0:dt:tf;
X0 =[X_eq
    U_eq(2:4)];
X = ode4(@PitchSAS_wind,T,X0,U_eq,aircraft,trim_par.Vw_v,X_eq,Y_eq);
Y= zeros (length (T), length(Y_eq)); 
U= zeros (length (T), length(U_eq));
% Vw= zeros (length(T),3);
Ucom = zeros (length (T), length(U_eq));
Upilot = zeros (length (T), length(U_eq));
Usas = zeros (length (T), length(U_eq));
Vw = zeros(length(T),3);
for i_t=1:length(T)
    [~,Yt,Ut,Ucomt,Upilott,Usast,Vwt] = PitchSAS_wind(T(i_t),X(i_t,:).',U_eq,aircraft,trim_par.Vw_v,X_eq,Y_eq);
    Y(i_t,:)= Yt.';
    U(i_t,:)= Ut.';
    Ucom(i_t,:)= Ucomt.';
    Upilot(i_t,:)= Upilott.';
    Usas(i_t,:)= Usast.';
    Vw(i_t,:) = Vwt.';
end

% ----------------------------------------------------------------------------------
% %% Linearization with atuator
% 
% X0 =[X_eq
%     U_eq(2:4)];
% % 
% % A = zeros(length (X_eq),length (X_eq));
% % B = zeros(length (X_eq),length (U_eq));
% % C = zeros(length (Y_eq),length (X_eq));
% % D = zeros(length (Y_eq),length (U_eq));
% 
% h=1e-5;
% 
% for i_X=1:length(X0)
%     dX = zeros(length(X0),1);
%     dX(i_X)= h;
%     [Xdot_plus, Y_plus]= control_input(0,X0+dX,U_eq,aircraft);
%     [Xdot_minus, Y_minus]= control_input(0,X0-dX,U_eq,aircraft);
%     Aa(:,i_X) = (Xdot_plus-Xdot_minus)/(2*dX(i_X));
%     Ca(:,i_X) = (Y_plus-Y_minus)/(2*dX(i_X));
% end
% 
% 
% for i_U=1:length(U_eq)
%     dU = zeros(length(U_eq),1);
%     dU(i_U)= h;
%     [Xdot_plus, Y_plus]= control_input(0,X0,U_eq+dU,aircraft);
%     [Xdot_minus, Y_minus]= control_input(0,X0,U_eq-dU,aircraft);
%     Ba(:,i_U) = (Xdot_plus-Xdot_minus)/(2*dU(i_U));
%     Da(:,i_U) = (Y_plus-Y_minus)/(2*dU(i_U));
% end
% 
% save ABCD_CGnominal.mat Aa Ba Ca Da




%% ---------------------------------------------------------------
close all
plot_all


%% -----------------------------------------------------------------
 close all
 visualizator_AB266(X)


figure
plot(T,Upilot(:,2))
hold all
plot(T,Usas(:,2))
xlabel('T')
ylabel('U')
legend('Upilot','Usas')
