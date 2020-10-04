clear
clc
close all


load trim_results_CG_multi_body.mat
Mat_X_eq_01=Mat_X_eq;
Mat_U_eq_01=Mat_U_eq;
Mat_Y_eq_01=Mat_Y_eq;
Mat_V_eq_01=Mat_V_eq;

load trim_results_CG_uni_body.mat
Mat_X_eq_02=Mat_X_eq;
Mat_U_eq_02=Mat_U_eq;
Mat_Y_eq_02=Mat_Y_eq;
Mat_V_eq_02=Mat_V_eq;
    
%% --------------------------------------------------------------
    figure
    subplot(2,1,1)
    hold on
    for i=1:length(Mat_U_eq)
    plot(Mat_V_eq_01{i}(:),Mat_U_eq_01{i}(1,:)*100)
%     plot(Mat_V_eq_02{i}(:),Mat_U_eq_02{i}(1,:)*100)
    end
    xlabel('V (m/s)')
    ylabel('Thrust (%)')
    legend('Multi-body','Uni-body')
    
    subplot(2,1,2)
    hold on 
    for i=1:length(Mat_U_eq)
    plot(Mat_V_eq_01{i}(:),Mat_U_eq_01{i}(2,:))
%     plot(Mat_V_eq_02{i}(:),Mat_U_eq_02{i}(2,:))
    end
    xlabel('V (m/s)')
    ylabel('Elevator (º)')
    axis([0 20 -20 20 ])


    
    figure
    subplot(2,1,1)
    hold on
    for i=1:length(Mat_U_eq)
    plot(Mat_V_eq_01{i}(:),Mat_X_eq_01{i}(13,:)*180/pi)
%     plot(Mat_V_eq_02{i}(:),Mat_X_eq_02{i}(13,:)*180/pi)
    end
    xlabel('V (m/s)')
    ylabel('Sigma (º)')
    legend('Multi-body','Uni-body')

    subplot(2,1,2)
    hold on
    for i=1:length(Mat_U_eq)
    plot(Mat_V_eq_01{i}(:),Mat_Y_eq_01{i}(8,:))
%     plot(Mat_V_eq_02{i}(:),Mat_Y_eq_02{i}(2,:))
    end
    xlabel('V (m/s)')
    ylabel('alpha (º)')
    
    

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