clear
clc
close all


% load Trim_results_multi_body/trim_results_final_crr.mat
load Multi_Body.mat 
Mat_X_eq_01=Mat_X_eq;
Mat_U_eq_01=Mat_U_eq;
Mat_Y_eq_01=Mat_Y_eq;
Mat_V_eq_01=Mat_V_eq;

    
%% --------------------------------------------------------------
    figure
    subplot(2,1,1)
    hold on
    plot(Mat_V_eq_01(:),Mat_U_eq_01(1,:)*100,'k')
    xlabel('V (m/s)')
    ylabel('Thrust (%)')
    legend('Multi-body','Uni-body')
    
    subplot(2,1,2)
    hold on 
    plot(Mat_V_eq_01(:),Mat_U_eq_01(2,:),'k')
    xlabel('V (m/s)')
    ylabel('Elevator (º)')
    axis([0 20 -20 20 ])


    
    figure
    subplot(2,1,1)
    hold on
    plot(Mat_V_eq_01(:),Mat_X_eq_01(13,:)*180/pi,'k')
    xlabel('V (m/s)')
    ylabel('Tilt Angle (º)')
    legend('Multi-body','Uni-body')

    subplot(2,1,2)
    hold on
    plot(Mat_V_eq_01(:),Mat_Y_eq_01(8,:),'k')
    xlabel('V (m/s)')
    ylabel('alpha (º)')

    figure
    subplot(2,1,1)
    hold on
    plot(Mat_V_eq_01(:),Mat_U_eq_01(3,:),'k')
    xlabel('V (m/s)')
    ylabel('Ailerton (º)')
    legend('Multi-body','Uni-body')

    subplot(2,1,2)
    hold on
    plot(Mat_V_eq_01(:),Mat_U_eq_01(4,:),'k')
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