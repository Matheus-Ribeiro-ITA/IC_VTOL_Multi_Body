clear
close all
clc

load ABCD_CGfwd_250fps_1500ft.mat Aa Ba Ca Da

sel_long = [1:4 13:14];% 1u 2 w, 3 q, 4 theta ,13 motor e 14 profudor
sel_ctrl = 1:2;
Along =Aa(sel_long,sel_long);
Blong = Ba(sel_long,sel_ctrl);
Clong = Ca(:,sel_long);
Dlong = Da(:,sel_ctrl);

V_eq = 250*0.3048;

Ctheta=[0 0 0 1 0 0];
Calpha=Clong(2,:);

A=[Along zeros(size(Along,1),1)
    V_eq*(Ctheta-Calpha)*pi/180 0];
B= [ Blong
    zeros(1,size(Blong,2))];

% Inner loops: [q theta]
Cq =[0 0 1 0 0 0];  % Recupera Theta e Q (das malhas internas??)
C=[Cq 0
    Ctheta 0];

% Outer loop: [V d]
CV = Clong(1,:);
Cd = [zeros(1,6) 1];
H=[CV 0
    Cd];

% Compensator dynamics
% yV = [eV xV]
FV=-5;
GV=1;
DV=[0;1];
JV=[1;0];

% yd=[xd epsd ed]
% states: [epsd xd]
Fd=[0 0; 1 -10];
Gd=[1; 0];
Dd=[0 1; 1 0;0 0];
Jd=[0;0;1];

% states: [xV epsd xd]
% outputs: [eV xV xd epsd ed]
F = [FV zeros(size(FV,1),size(Fd,2))
    zeros(size(Fd,1),size(FV,2)) Fd];
G = [GV zeros(size(GV,1),size(Gd,2))
    zeros(size(Gd,1),size(GV,2)) Gd];
D = [DV zeros(size(DV,1),size(Dd,2))
    zeros(size(Dd,1),size(DV,2)) Dd];
J = [JV zeros(size(JV,1),size(Jd,2))
    zeros(size(Jd,1),size(JV,2)) Jd];   

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

q=0.5;
Q=q*Q;


R = [1/(0.1^2) 0
    0 1/(10^2)];

V = [1/(0.01^2) 0 
    0 0]; % 

fK = @(k)([0 0 k(3:4) 0 0 0; k(1:2) 0 0 k(5:7)]);
fPI = @(k)(PI_Tracker_t2(fK(k),Aa,Ba,Ca,Fa,Ga,Ha,Q,R,V));
% fPI_Integration = @(k)(PI_LQR_Integration(fK(k),A_LQR,B_LQR,C_LQR,Q,R));
fMaxReal = @(k)(max_eig_real(fK(k),Aa,Ba,Ca));

k0 =zeros(1,7);
fMaxReal(k0)

fStabGain = @(k)(fK(k(1:7)));
fMaxRealStabGain = @(k)(max_eig_real(fStabGain(k),Aa,Ba,Ca));
options = optimset('Display','iter',...
    'MaxFunEvals',1e4,'Maxiter',1e3);
k0 =fminsearch(fMaxRealStabGain,-ones(1,7),options);

options = optimoptions(@fmincon,...
    'Algorithm','interior-point','Display','iter',...
    'MaxFunEvals',1e4,'Maxiter',1e3);

k_sol =fmincon(fPI,k0,[],[],[],[],[],[],fMaxReal,options);

K = fK(k_sol)

% % foto
% PI_Q = PI_Tracker_t2(fk(k_sol),Aa,Ba,Ca,Fa,Ga,Ha,Q,0*R,)


Ac=Aa- Ba*K*Ca;
Bc=Ga- Ba*K*Fa;

% Response to unit steps
figure
subplot(211)
ss_Vc_2_V = ss(Ac,Bc(:,1),Ha(1,:),0);
[y,t,x] = step(ss_Vc_2_V,0:0.010:20);
Vss = evalfr(ss_Vc_2_V,0)
plot(t,y)
xlabel('t [s]')
ylabel('Delta V [m/s]')


subplot(212)
ss_dc_2_d = ss(Ac,Bc(:,2),Ha(2,:),0);
[y,t,x] = step(ss_dc_2_d,0:0.010:20);
Vss = evalfr(ss_dc_2_d,0);
plot(t,y)
xlabel('t [s]')
ylabel('Delta d [m/s]')

% Linear simulatio of glide slope coupler:
gamma_r_deg = -2.5;

Bsim = [zeros(size(Along,1),1)
        V_eq*(-gamma_r_deg)*pi/180
        zeros(3,1)];
Csim = [Ctheta-Calpha 0 0 0 0
    CV 0 0 0 0
    Cd 0 0 0
    -K*Ca];

ss_glide_slope=ss(Ac,Bsim,Csim,0);

figure
[y,t,x]= step(ss_glide_slope,0:0.010:20);

%%
plot(t,x(:,1))
xlabel('t [s]')
ylabel('power')

% % C-star criterion:
% 
% Ac=Aa-Ba*K*Ca;
% Bc = Ga-Ba*K*F    a;
% CCstar = Ha+12.4*pi/180*Ca(1,:);
% ss_cl_Cstar = ss(Ac,Bc,CCstar,0);
% Yss = evalfr(ss_cl_Cstar,0)
% [Y,T]=step(ss_cl_Cstar,0:0.010:3);
% figure
% plot(T,Y)
% xlabel('t [s]')
% ylabel('C*')
% 
% CStar_plot
% hold on 
% plot(T,Y/Yss,'b--','LineWidth',1.5)
% 
% % Inner loop, q feedback
% C_q =[0 1 0];
% ss_ue_deg_2_q_deg_s = ss(Along, Blong,C_q,0)
% 
% k = -logspace(-2,1,500); % vetor de 10^-2
% figure
% rlocus(ss_ue_deg_2_q_deg_s,k)
% 
% k_q=-0.4;
% 
% Acl_q = Along-Blong*k_q*C_q;
% 
% % Pure Integrator
% ss_I = ss(0,1,1,1);
% ss_ue_deg_2_nzp = ss(Acl_q, Blong,Cnzp,Dnzp);
% ss_rc_2_nzp = series(ss_I,ss_ue_deg_2_nzp);
% 
% figure
% rlocus(ss_rc_2_nzp,k)
% 
% % PI compensator
% z=0.9;
% ss_PI = ss(0,z,1,1);
% ss_ue_deg_2_nzp = ss(Acl_q, Blong,Cnzp,Dnzp);
% ss_rc_2_nzp = series(ss_PI,ss_ue_deg_2_nzp);
% figure
% rlocus(ss_rc_2_nzp,k)
% k_p=-5;
% 
% [aa, bb, cc, dd] =ssdata(ss_rc_2_nzp);
% ss_cl_nzCAS = ss(aa-bb*k_p*cc,bb*k_p,cc,dd*k_p);
% 
% figure
% step(ss_cl_nzCAS,0:0.01:10)
% 
% % C-star criterion:
% Cq = [0 1 0 0]*pi/180;
% ss_Cstar =ss(aa-bb*k_p*cc,bb*k_p,cc + 12.4*Cq,dd*k_p);
% tf_Cstar = tf(ss_Cstar);
% Yss = evalfr(tf_Cstar,0); % Avalia a função de tranferencia em 0, teorema do valor final
% [Y,T]= step(ss_Cstar,0:0.010:3);
% figure
% plot (T,Y);
% xlabel('t [s]');
% ylabel('C*');
% 
% CStar_plot
% hold on
% plot(T,Y/Yss,'b--','LineWidth',1.5)
% 
% % Test on the full dynamics:
% Cstate = eye(size (Aa,1));
% C_q =Cstate(3,:);
% Acl_q = Aa - Ba(:,2)*k_q*C_q;
% C_nzp = Ca(i_nzp,:);
% D_nzp = Da(i_nzp,:);
% ss_ue_deg_2_nz = ss(Acl_q,Ba(:,2),C_nzp,D_nzp(:,2));
% ss_rc_2_nzp = series(ss_PI,ss_ue_deg_2_nzp);
% [aa, bb, cc, dd] =ssdata(ss_rc_2_nzp);
% Acl_nzCAS = aa-bb*k_p*cc;
% ss_cl_nzCAS = ss(Acl_nzCAS,bb*k_p,cc,dd*k_p);
% 
% figure
% step(ss_cl_nzCAS,0:0.01:10)
% 
% figure
% plot(real(eig(Aa)),imag(eig(Aa)),'b*','MarkerSize',10)
% hold on
% plot(real(eig(Acl_nzCAS)),imag(eig(Acl_nzCAS)),'rx','MarkerSize',10)
% grid on
% legend('Open loop', 'Closed loop')
% 
% 
% 
% 
% 
% 
% 
