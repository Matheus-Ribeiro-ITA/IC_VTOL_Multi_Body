clear
clc
close all


load Trim_results_uni_body/trim_results_opt_02.mat
Mat_X_eq_01=Mat_X_eq;
Mat_U_eq_01=Mat_U_eq;
Mat_Y_eq_01=Mat_Y_eq;
Mat_V_eq;
    
%% --------------------------------------------------------------
    figure
    subplot(2,1,1)
    plot(Mat_V_eq(:),Mat_U_eq_01(1,:)*100)
    xlabel('V (m/s)')
    ylabel('Thrust (%)')
    
    
    subplot(2,1,2)
    plot(Mat_V_eq(:),Mat_U_eq_01(2,:))
    xlabel('V (m/s)')
    ylabel('Elevator (º)')



    
    figure
    subplot(2,1,1)
    plot(Mat_V_eq(:),Mat_X_eq_01(13,:)*180/pi)
    xlabel('V (m/s)')
    ylabel('Sigma (º)')


    subplot(2,1,2)
    plot(Mat_V_eq(:),Mat_Y_eq_01(2,:))
    xlabel('V (m/s)')
    ylabel('alpha (º)')

 %% --------------------------------------------------------------
    figure
    subplot(2,1,1)
    hold on
    plot(Mat_V_eq(:),Mat_Y_eq_01(14,:)/9.8,'DisplayName','Aero.')
    plot(Mat_V_eq(:),Mat_Y_eq_01(17,:)/9.8,'DisplayName','Prop.')
    xlabel('V (m/s)')
    ylabel('F_z aero (kgf)')
    legend
    
    
    subplot(2,1,2)
    plot(Mat_V_eq(:),Mat_Y_eq_01(5,:))
    xlabel('V (m/s)')
    ylabel('Cm ')