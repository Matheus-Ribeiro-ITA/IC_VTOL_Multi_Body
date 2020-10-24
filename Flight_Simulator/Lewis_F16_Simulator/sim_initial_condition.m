
% Simulation of initial condition:
C_sim = [C_LQR
    0 0 0 0 1 0 0
    0 0 0 0 0 1 0
    -K*C_LQR
    -K_GF*C_LQR];
%Recuperando: phi p beta rW da dr ua_K ur_K ua_K_GF ur_K_GF  
ss_OL = ss(Aldw,Bldw,C_sim,zeros(size(C_sim,1),size(Bldw,2)));
ss_CL_K = ss(A_LQR-B_LQR*K*C_LQR,Bldw,C_sim,zeros(size(C_sim,1),size(Bldw,2)));
ss_CL_K_GF = ss(A_LQR-B_LQR*K_GF*C_LQR,Bldw,C_sim,zeros(size(C_sim,1),size(Bldw,2)));

% beta(0) = 3 deg;
x0 = [502*0.3048*sin(3*pi/180); zeros(6,1)]; % 502feet/s feet2m sin(beta)

[y_OL,t_OL,x_OL] = initial(ss_OL,x0,0:0.010:20);
[y_CL_K,t_CL_K,x_CL_K] = initial(ss_CL_K,x0,0:0.010:20);
[y_CL_K_GF,t_CL_K_GF,x_CL_K_GF] = initial(ss_CL_K_GF,x0,0:0.010:20);

figure
plot(t_OL,y_OL(:,3),'LineWidth',2)
hold all
plot(t_CL_K,y_CL_K(:,3),'LineWidth',2)
plot(t_CL_K_GF,y_CL_K_GF(:,3),'LineWidth',2)
grid on
xlabel('t [s]')
ylabel('\beta [deg]')
legend('OL','CL K','CL K GF')

figure
subplot(221)
plot(t_CL_K,y_CL_K(:,5),'LineWidth',2)
hold all
plot(t_CL_K_GF,y_CL_K_GF(:,5),'LineWidth',2)
grid on
xlabel('t [s]')
ylabel('\delta_a [deg]')

subplot(222)
plot(t_CL_K,y_CL_K(:,7),'LineWidth',2)
hold all
plot(t_CL_K_GF,y_CL_K_GF(:,9),'LineWidth',2)
grid on
xlabel('t [s]')
ylabel('u_a [deg]')

subplot(223)
plot(t_CL_K,y_CL_K(:,6),'LineWidth',2)
hold all
plot(t_CL_K_GF,y_CL_K_GF(:,6),'LineWidth',2)
grid on
xlabel('t [s]')
ylabel('\delta_r [deg]')

subplot(224)
plot(t_CL_K,y_CL_K(:,8),'LineWidth',2)
hold all
plot(t_CL_K_GF,y_CL_K_GF(:,10),'LineWidth',2)
grid on
xlabel('t [s]')
ylabel('u_r [deg]')
legend('CL K','CL K GF')
