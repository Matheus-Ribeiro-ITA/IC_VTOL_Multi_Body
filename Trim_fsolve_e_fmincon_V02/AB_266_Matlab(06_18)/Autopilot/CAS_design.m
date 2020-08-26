clear
close all
clc

load ABCD_CGnominal.mat Aa Ba Ca Da
% Aa Matriz A com atuador
sel_long = [2:3 14];% 2 w, 3 q , 14 profudor
sel_ctrl = 2; % u e ??
Along =Aa(sel_long,sel_long);
Blong = Ba(sel_long,sel_ctrl);
Clong = Ca(:,sel_long);
Dlong = Da(:,sel_ctrl);

i_nzp = size(Ca,1);
i_nzc = size(Ca,1)-3;

Cnzp = Clong(i_nzp,:)
Dnzp = Dlong(i_nzp,:)

A=Along;
B=Blong;
C= [0 1 0]; % Segundo estado é q

F=0;
G=1;
D=[0 ; 1];
J=[1 ; 0];

H = Cnzp;

Aa = [A zeros(size(A,1),size(F,2))
     -G*H F];
Ba = [B
zeros(size(F,1),size(B,2))];
Ga= [zeros(size(A,1),size(G,2))
    G];
Ca=[C zeros(size(C,1),size(D,2))
    -J*H D];
Fa =[zeros(size(C,1),size(J,2))
    J];
Ha= [H zeros(size(H,1),size(F,2))];

Q = Ha.'*Ha;
% obsev(Aa,Ha);
% rank(obsev(Aa,Ha)); % Posto menor que numero de colunas (LD)
Q(end,end)=1; % Problema de Posto da matriz q

% obsev(Aa,sqrtm(Q));
% rank(obsev(Aa,sqrtm(Q)));

R = 1/(5^2);

V = 1;

fK = @(k)(k(1:3));
fPI = @(k)(PI_Tracker(fK(k),Aa,Ba,Ca,Fa,Ga,Ha,Q,R,V));
% fPI_Integration = @(k)(PI_LQR_Integration(fK(k),A_LQR,B_LQR,C_LQR,Q,R));
fMaxReal = @(k)(max_eig_real(fK(k),Aa,Ba,Ca));

% k0 =zeros(1,3);
k0 = [-0.4 5 5*0.9];
% Valores da semana passada:
%k_q= -0.4;
% z= 0.9;
% k_p= 5;

fMaxReal(k0)

options = optimoptions(@fmincon,...
    'Algorithm','sqp','Display','iter',...
    'MaxFunEvals',1e4,'Maxiter',1e3);

k_sol =fmincon(fPI,k0,[],[],[],[],[],[],fMaxReal,options);

K = fK(k_sol)

% C-star criterion:

Ac=Aa-Ba*K*Ca;
Bc = Ga-Ba*K*Fa;
CCstar = Ha+12.4*pi/180*Ca(1,:);
ss_cl_Cstar = ss(Ac,Bc,CCstar,0);
Yss = evalfr(ss_cl_Cstar,0)
[Y,T]=step(ss_cl_Cstar,0:0.010:3);
figure
plot(T,Y)
xlabel('t [s]')
ylabel('C*')

CStar_plot
hold on 
plot(T,Y/Yss,'b--','LineWidth',1.5)

% Inner loop, q feedback
C_q =[0 1 0];
ss_ue_deg_2_q_deg_s = ss(Along, Blong,C_q,0)

k = -logspace(-2,1,500); % vetor de 10^-2
figure
rlocus(ss_ue_deg_2_q_deg_s,k)

k_q=-0.4;

Acl_q = Along-Blong*k_q*C_q;

% Pure Integrator
ss_I = ss(0,1,1,1);
ss_ue_deg_2_nzp = ss(Acl_q, Blong,Cnzp,Dnzp);
ss_rc_2_nzp = series(ss_I,ss_ue_deg_2_nzp);

figure
rlocus(ss_rc_2_nzp,k)

% PI compensator
z=0.9;
ss_PI = ss(0,z,1,1);
ss_ue_deg_2_nzp = ss(Acl_q, Blong,Cnzp,Dnzp);
ss_rc_2_nzp = series(ss_PI,ss_ue_deg_2_nzp);
figure
rlocus(ss_rc_2_nzp,k)
k_p=-5;

[aa, bb, cc, dd] =ssdata(ss_rc_2_nzp);
ss_cl_nzCAS = ss(aa-bb*k_p*cc,bb*k_p,cc,dd*k_p);

figure
step(ss_cl_nzCAS,0:0.01:10)

% C-star criterion:
Cq = [0 1 0 0]*pi/180;
ss_Cstar =ss(aa-bb*k_p*cc,bb*k_p,cc + 12.4*Cq,dd*k_p);
tf_Cstar = tf(ss_Cstar);
Yss = evalfr(tf_Cstar,0); % Avalia a função de tranferencia em 0, teorema do valor final
[Y,T]= step(ss_Cstar,0:0.010:3);
figure
plot (T,Y);
xlabel('t [s]');
ylabel('C*');

CStar_plot
hold on
plot(T,Y/Yss,'b--','LineWidth',1.5)

% Test on the full dynamics:
Cstate = eye(size (Aa,1));
C_q =Cstate(3,:);
Acl_q = Aa - Ba(:,2)*k_q*C_q;
C_nzp = Ca(i_nzp,:);
D_nzp = Da(i_nzp,:);
ss_ue_deg_2_nz = ss(Acl_q,Ba(:,2),C_nzp,D_nzp(:,2));
ss_rc_2_nzp = series(ss_PI,ss_ue_deg_2_nzp);
[aa, bb, cc, dd] =ssdata(ss_rc_2_nzp);
Acl_nzCAS = aa-bb*k_p*cc;
ss_cl_nzCAS = ss(Acl_nzCAS,bb*k_p,cc,dd*k_p);

figure
step(ss_cl_nzCAS,0:0.01:10)

figure
plot(real(eig(Aa)),imag(eig(Aa)),'b*','MarkerSize',10)
hold on
plot(real(eig(Acl_nzCAS)),imag(eig(Acl_nzCAS)),'rx','MarkerSize',10)
grid on
legend('Open loop', 'Closed loop')







