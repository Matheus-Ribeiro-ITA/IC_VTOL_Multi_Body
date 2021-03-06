clear all
close all
clc

load ABCD_CGnominal.mat Aa Ba Ca Da

sel_long = [1:5 13:14];% 13 motor, 14 profudor
sel_ctrl = 1:2;
Along =Aa(sel_long,sel_long);
Blong = Ba(sel_long,:);
Clong = Ca(:,sel_long);
Dlong = Da(:,sel_ctrl);

ss_ue_deg_2_alpha_deg = ss(Along,Blong(:,2),Clong(2,:),Dlong(2,2)); %Ssitema de espaco e estados comando de profundor para alpha
tf_ue_deg_2_alpha_deg = tf(ss_ue_deg_2_alpha_deg)
zpk_ue_deg_2_alpha_deg = zpk(ss_ue_deg_2_alpha_deg)
figure
pzmap(ss_ue_deg_2_alpha_deg)

k = -logspace(-2,1,500); % vetor de 10^-2
figure
rlocus(ss_ue_deg_2_alpha_deg,k)

k_alpha = -0.6;

Acl_alpha = Along-Blong(:,2)*k_alpha*Clong(2,:);
damp(Acl_alpha)














k_q=-0.2;

% Closed-loop with alpha and q feedback
Acl_alpha_q= Acl_alpha-Blong(:,2)*k_q*C_q;
damp(Acl_alpha_q)

figure
plot(real(eig(Along)),imag(eig(Along)),'b*')
hold on
plot(real(eig(Acl_alpha_q)),imag(eig(Acl_alpha_q)),'rs')
% sgrid
legend('Longitudinal open loop','Longitudinal close loop','Location','best')

% Closed-loop with alpha and q feedback, all states:
Cstate = eye(size(Aa,1));
C_fdbck