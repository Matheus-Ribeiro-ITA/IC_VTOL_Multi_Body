clearvars
close all
clc

load ABCD_CGnominal.mat

sel_latdir = [7:10 15:16];%7= V_y, 8=Phi, 9=p, 10=r, 15=ail, 16= rudder
sel_ctrl = 3:4;

Ald =Aa(sel_latdir,sel_latdir);
Bld =Ba(sel_latdir,sel_ctrl);
Cld =Ca(:,sel_latdir);
Dld =Da(:,sel_ctrl);

% series (sys1,sys2) % Nao necessiaramente nessa ordem 

tau_W =1;
Aldw =[Ald zeros(size(Ald,1),1)
    0 0 0 -1/tau_W 0 0 -1/tau_W];
Bldw =[Bld
    zeros(1,2) ];
Cldw =[Cld zeros(size(Cld,1),1)
    0 0 0 1 0 0 1];
Dldw =[Dld
    zeros(1,2)];

damp(Ald)
damp(Aldw)

A_LQR = Aldw;
B_LQR = Bldw;
C_LQR = [0 1 0 0 0 0 0
    0 0 1 0 0 0 0
    Cldw(3,:)
    Cldw(22,:)]; % r washout
% y : phi_deg, p_deg_s, beta_deg, r_W_deg_s


for q=2:1:20

Q= zeros (4,4);
Q(1,1) = 1/(q^2);
Q(2,2) = 1/(q^2);
Q(3,3) = 1/(q^2);
Q(4,4) = 1/(q^2);

Q = C_LQR.'*Q*C_LQR;

R = zeros(2,2);
R(1,1) =1/(5^2);
R(2,2) =1/(5^2);

fK = @(k)([k(1:4); k(5:8)]);
fPI = @(k)(PI_LQR(fK(k),A_LQR,B_LQR,C_LQR,Q,R));
% fPI_Integration = @(k)(PI_LQR_Integration(fK(k),A_LQR,B_LQR,C_LQR,Q,R));
fMaxReal = @(k)(max_eig_real(fK(k),A_LQR,B_LQR,C_LQR));

k0 =zeros(1,8);
fMaxReal(k0)

options = optimoptions(@fmincon,...
    'Algorithm','sqp','Display','iter',...
    'MaxFunEvals',1e4,'Maxiter',1e3);

k_sol =fmincon(fPI,k0,[],[],[],[],[],[],fMaxReal,options);

K = fK(k_sol)


Ac = A_LQR - B_LQR*K*C_LQR;
damp(Ac)
[V,~]=eig(Ac)

figure
plot(eig(A_LQR),'bx','MarkerSize',10)
hold on
plot (eig(Ac),'rs') 
legend('OL','CL','Location','Best')
grid on

J_optim = fPI(k_sol)
% J_integr = fPI_Integration(k_sol)

% Gain element weighting
g=1e4;
GW = [0 0 g g
      g g 0 0];
fPI_GW = @(k)(PI_LQR_GW(fK(k),A_LQR,B_LQR,C_LQR,Q,R,GW));
k_sol =fmincon(fPI_GW,k0,[],[],[],[],[],[],fMaxReal,options);
K_GW = fK(k_sol)

% Gain element fixing
fK_GF = @(k)([k(1:2) 0 0 ; 0 0 k(3:4)]);
fPI_GF = @(k)(PI_LQR_GW(fK_GF(k),A_LQR,B_LQR,C_LQR,Q,R,GW));
fMaxReal_GF = @(k)(max_eig_real(fK_GF(k),A_LQR,B_LQR,C_LQR));

k0 =zeros(1,4);
k_sol =fmincon(fPI_GF,k0,[],[],[],[],[],[],fMaxReal_GF,options);
K_GF = fK_GF(k_sol)

Ac = A_LQR - B_LQR*K_GF*C_LQR;
damp(Ac)
[V,~]=eig(Ac)

figure
plot(eig(A_LQR),'bx','MarkerSize',10)
hold on
plot (eig(Ac),'rs') 
legend('OL','CL','Location','Best')
grid on

end