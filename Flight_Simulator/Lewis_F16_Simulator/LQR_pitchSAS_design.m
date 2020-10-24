clearvars
close all
clc

load ABCD_CGnominal.mat

sel_long = [1:4 14];
sel_ctrl = 2;
Along=Aa(sel_long,sel_long);
Blong=Ba(sel_long,sel_ctrl);
Clong = Ca(:,sel_long);
Dlong = Da(:,sel_ctrl);

%damp(Aa)
%damp(Along)

A_LQR=Along;
B_LQR=Blong;
C_LQR=[Clong(2,:) 
    0 0 1 0 0];
% y: alpha_deg, q_deg_s

Q=zeros(2,2);
Q(1,1)=1/(2^2);
Q(2,2)=1/(5^2);
Q= C_LQR.'*Q*C_LQR; 

R =1/(2^2);

fK = @(k)([k(1) k(2)]);
fPI = @(k)(PI_LQR(fK(k),A_LQR,B_LQR,C_LQR,Q,R));
% fPI_Integration = @(k)(PI_LQR_Integration(fK(k),A_LQR,B_LQR,C_LQR,Q,R));
fMaxReal = @(k)(max_eig_real(fK(k),A_LQR,B_LQR,C_LQR));

% Finding initial stabilized gains

ss_ue_2_alpha_deg = ss(A_LQR,B_LQR,C_LQR(1,:),0);
k= -logspace(-2,1,500);
figure
rlocus(ss_ue_2_alpha_deg,k)

k_alpha = -0.2;

k0 =[k_alpha 0];

options = optimoptions(@fmincon,...
    'Algorithm','sqp','Display','iter',...
    'MaxFunEvals',1e4,'Maxiter',1e3);

k_sol =fmincon(fPI,k0,[],[],[],[],[],[],fMaxReal,options);


K = fK(k_sol)


Ac = A_LQR - B_LQR*K*C_LQR;
figure
plot(eig(A_LQR),'bx','MarkerSize',10)
hold on
plot (eig(Ac),'rs') 
legend('OL','CL','Location','Best')
grid on

J_optim = fPI(k_sol)
% J_integr = fPI_Integration(k_sol)