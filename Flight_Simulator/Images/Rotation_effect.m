clear
clc
close all


% load ../Trim_results_Rudder/trim_results_opt_01_crr.mat 
load Rotation_Multi_Body_crr.mat 

% Rotarion speed
% sigma_dot= 10*pi/180;
% sigma_dot_dot= 5*pi/180;

Mat_X_eq_01=Mat_X_eq;
Mat_U_eq_01=Mat_U_eq;
Mat_Y_eq_01=Mat_Y_eq;
Mat_V_eq_01=Mat_V_eq;

% load ../Trim_results_uni_body/trim_results_opt_03.mat
load Uni_body.mat
Mat_X_eq_02=Mat_X_eq;
Mat_U_eq_02=Mat_U_eq;
Mat_Y_eq_02=Mat_Y_eq;
Mat_V_eq_02=Mat_V_eq;
    
%% --------------------------------------------------------------
    quebra = 31;

    figure
    subplot(2,1,1)
    hold on
    plot(Mat_V_eq_01(1:quebra),Mat_U_eq_01(1,1:quebra)*100,'r--')
    plot(Mat_V_eq_02(:),Mat_U_eq_02(1,:)*100,'k')
    plot(Mat_V_eq_01(quebra+1:end),Mat_U_eq_01(1,quebra+1:end)*100,'r--')
    xlabel('V (m/s)')
    ylabel('Thrust (%)')
    legend('Multi-body','Uni-body')
    
    subplot(2,1,2)
    hold on 
    plot(Mat_V_eq_01(1:quebra),Mat_U_eq_01(2,1:quebra),'r--')
    plot(Mat_V_eq_02(:),Mat_U_eq_02(2,:),'k')
    plot(Mat_V_eq_01(quebra+1:end),Mat_U_eq_01(2,quebra+1:end),'r--')
    xlabel('V (m/s)')
    ylabel('Elevator (º)')
    axis([0 20 -20 20 ])


    
    figure
    subplot(2,1,1)
    hold on
    plot(Mat_V_eq_01(:),Mat_X_eq_01(13,:)*180/pi,'r--')
    plot(Mat_V_eq_02(:),Mat_X_eq_02(13,:)*180/pi,'k')
    xlabel('V (m/s)')
    ylabel('Tilt Angle (º)')
    legend('Multi-body','Uni-body')

    subplot(2,1,2)
    hold on
    plot(Mat_V_eq_01(:),Mat_Y_eq_01(8,:),'r--')
    plot(Mat_V_eq_02(:),Mat_Y_eq_02(2,:),'k')
    xlabel('V (m/s)')
    ylabel('alpha (º)')

    figure
    subplot(2,1,1)
    hold on
    plot(Mat_V_eq_01(:),Mat_U_eq_01(3,:),'r--')
    plot(Mat_V_eq_02(:),Mat_U_eq_02(3,:),'k')
    xlabel('V (m/s)')
    ylabel('Aileron (º)')
    legend('Multi-body','Uni-body')

    subplot(2,1,2)
    hold on
    plot(Mat_V_eq_01(1:quebra),Mat_U_eq_01(4,1:quebra),'r--')
    plot(Mat_V_eq_02(:),Mat_U_eq_02(4,:),'k')
    plot(Mat_V_eq_01(quebra+1:end),Mat_U_eq_01(4,quebra+1:end),'r--')
    xlabel('V (m/s)')
    ylabel('Rudder (º)')
    
    

 %% --------------------------------------------------------------
%     figure
%     subplot(2,1,1)
%     hold on
%     plot(Mat_V_eq(:),Mat_Y_eq_01(14,:)/9.8,'DisplayName','Aero.')
%     plot(Mat_V_eq(:),Mat_Y_eq_01(17,:)/9.8,'DisplayName','Prop.')
%     xlabel('V (m/s)')
%     ylabel('F_z aero (kgf)')
%     legend
%     
%     
%     subplot(2,1,2)
%     plot(Mat_V_eq(:),Mat_Y_eq_01(5,:))
%     xlabel('V (m/s)')
%     ylabel('Cm ')