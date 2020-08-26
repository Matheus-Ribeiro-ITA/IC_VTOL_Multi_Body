clear
clc
close all


load trim_results_01.mat
Mat_X_eq_01=Mat_X_eq;
Mat_U_eq_01=Mat_U_eq;
Mat_Y_eq_01=Mat_Y_eq;

load trim_results_02.mat
Mat_X_eq_02=Mat_X_eq;
Mat_U_eq_02=Mat_U_eq;
Mat_Y_eq_02=Mat_Y_eq;

load trim_results_03.mat
Mat_X_eq_03=Mat_X_eq;
Mat_U_eq_03=Mat_U_eq;
Mat_Y_eq_03=Mat_Y_eq;

%% --------------------------------------------------------------
    figure
    title('V = 15 m/s')
    subplot(3,1,1)
    plot(Mat_X_eq_01(13,:)*180/pi,Mat_U_eq_01(1,:)*100, ... 
        Mat_X_eq_02(13,:)*180/pi,Mat_U_eq_02(1,:)*100, ...
        Mat_X_eq_03(13,:)*180/pi,Mat_U_eq_03(1,:)*100)
    xlabel('Sigma (º)')
    ylabel('Thrust (%)')
    legend ('ME=0%','ME=20%','ME=-20%')
    
    
    subplot(3,1,2)
    plot(Mat_X_eq_01(13,:)*180/pi,Mat_U_eq_01(2,:),...
        Mat_X_eq_02(13,:)*180/pi,Mat_U_eq_02(2,:),...
        Mat_X_eq_03(13,:)*180/pi,Mat_U_eq_03(2,:))
    xlabel('Sigma (º)')
    ylabel('Elevator (º)')
%     legend ('ME=0%','ME=20%')

    subplot(3,1,3)
    plot(Mat_X_eq_01(13,:)*180/pi,Mat_Y_eq_01(8,:), ...
        Mat_X_eq_02(13,:)*180/pi,Mat_Y_eq_02(8,:), ...
        Mat_X_eq_03(13,:)*180/pi,Mat_Y_eq_03(8,:))
    xlabel('Sigma (º)')
    ylabel('alpha (º)')
%     legend ('ME=0%','ME=20%')
    