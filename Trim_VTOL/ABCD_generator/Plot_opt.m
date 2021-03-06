clear
clc
close all


load trim_results_opt_08.mat
Mat_X_eq_01=Mat_X_eq;
Mat_U_eq_01=Mat_U_eq;
Mat_Y_eq_01=Mat_Y_eq;
Mat_V_eq

%% --------------------------------------------------------------
    figure
    subplot(2,1,1)
    plot(Mat_V_eq(:),Mat_U_eq_01(1,:)*100)
    xlabel('V (m/s)')
    ylabel('Thrust (%)')
    
    
    subplot(2,1,2)
    plot(Mat_V_eq(:),Mat_U_eq_01(2,:))
    xlabel('V (m/s)')
    ylabel('Elevator (�)')



    
    figure
    subplot(2,1,1)
    plot(Mat_V_eq(:),Mat_X_eq_01(13,:)*180/pi)
    xlabel('V (m/s)')
    ylabel('Sigma (�)')


    subplot(2,1,2)
    plot(Mat_V_eq(:),Mat_Y_eq_01(8,:))
    xlabel('V (m/s)')
    ylabel('alpha (�)')

 %% --------------------------------------------------------------
    figure
    subplot(2,1,1)
    plot(Mat_V_eq(:),Mat_Y_eq_01(3,:))
    xlabel('V (m/s)')
    ylabel('CL')
    
    
    subplot(2,1,2)
    plot(Mat_V_eq(:),Mat_Y_eq_01(5,:))
    xlabel('V (m/s)')
    ylabel('Cm ')