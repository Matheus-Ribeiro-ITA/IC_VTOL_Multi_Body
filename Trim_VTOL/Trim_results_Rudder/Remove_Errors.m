close all
clear
clc


load trim_results_opt_04.mat


figure
hold on
plot(Mat_X_eq(13,:)*180/pi)
xlabel('índice')
ylabel('Sigma (º)')
legend('Multi-body')

figure
subplot(2,1,1)
hold on
plot(Mat_V_eq(:),Mat_U_eq(1,:)*100)
xlabel('V (m/s)')
ylabel('Thrust (%)')
legend('Multi-body')

subplot(2,1,2)
hold on 
plot(Mat_V_eq(:),Mat_U_eq(2,:))
xlabel('V (m/s)')
ylabel('Elevator (º)')
axis([0 20 -20 20 ])

indice_remover = [40,39, 37, 36, 29, 28, 26, 24, 22];

for i=1:length(indice_remover)
    Mat_V_eq(:,indice_remover(i)) = [];
    Mat_U_eq(:,indice_remover(i)) = [];
    Mat_X_eq(:,indice_remover(i)) = [];
    Mat_Y_eq(:,indice_remover(i)) = [];
end



figure
subplot(2,1,1)
hold on
plot(Mat_V_eq(:),Mat_U_eq(1,:)*100)
xlabel('V (m/s)')
ylabel('Thrust (%)')
legend('Multi-body')

subplot(2,1,2)
hold on 
plot(Mat_V_eq(:),Mat_U_eq(2,:))
xlabel('V (m/s)')
ylabel('Elevator (º)')
axis([0 20 -20 20 ])

figure
subplot(2,1,1)
hold on
plot(Mat_V_eq(:),Mat_X_eq(13,:)*180/pi)
xlabel('V (m/s)')
ylabel('Sigma (º)')
legend('Multi-body')

subplot(2,1,2)
hold on
plot(Mat_V_eq(:),Mat_Y_eq(8,:))
xlabel('V (m/s)')
ylabel('alpha (º)')

figure
subplot(2,1,1)
hold on
plot(Mat_U_eq(3,:))
xlabel('V (m/s)')
ylabel('Aileron (º)')
legend('Multi-body')

subplot(2,1,2)
hold on
plot(Mat_U_eq(4,:))
xlabel('V (m/s)')
ylabel('Rudder (º)')

dev=3;
for i=27:30
    Mat_U_eq(3,i) = mean(Mat_U_eq(3,i-dev:i+dev));
    Mat_U_eq(4,i) = mean(Mat_U_eq(4,i-dev:i+dev));
end

Mat_U_eq(3,11) = mean([Mat_U_eq(3,10),Mat_U_eq(3,12)]);
Mat_U_eq(3,27) = mean([Mat_U_eq(3,25),Mat_U_eq(3,28)]);
Mat_U_eq(3,26) = mean([Mat_U_eq(3,25),Mat_U_eq(3,28)]);
Mat_U_eq(3,29) = mean([Mat_U_eq(3,28),Mat_U_eq(3,30)]);
Mat_U_eq(3,31) = mean([Mat_U_eq(3,32),Mat_U_eq(3,30)]);
Mat_U_eq(3,26) = mean([Mat_U_eq(3,25),Mat_U_eq(3,27)]);

Mat_U_eq(4,11) = mean([Mat_U_eq(4,10),Mat_U_eq(4,12)]);
Mat_U_eq(4,27) = mean([Mat_U_eq(4,25),Mat_U_eq(4,28)]);
Mat_U_eq(4,26) = mean([Mat_U_eq(4,25),Mat_U_eq(4,28)]);
Mat_U_eq(4,29) = mean([Mat_U_eq(4,28),Mat_U_eq(4,30)]);
Mat_U_eq(4,31) = mean([Mat_U_eq(4,32),Mat_U_eq(4,30)]);
Mat_U_eq(4,26) = mean([Mat_U_eq(4,25),Mat_U_eq(4,27)]);
Mat_U_eq(4,28) = Mat_U_eq(4,28)*1.99;
Mat_U_eq(4,29) = Mat_U_eq(4,29)*1.99;
Mat_U_eq(4,30) = Mat_U_eq(4,30)*1.99;
Mat_U_eq(4,31) = Mat_U_eq(4,31)*1.99 -0.02;
% Mat_U_eq(4,28) = 
% Mat_U_eq(4,27) =
% Mat_U_eq(4,31) = 

figure
subplot(2,1,1)
hold on
plot(Mat_U_eq(3,:))
xlabel('V (m/s)')
ylabel('Aileron (º)')
legend('Multi-body')

subplot(2,1,2)
hold on
plot(Mat_U_eq(4,:))
xlabel('V (m/s)')
ylabel('Rudder (º)')



figure
subplot(2,1,1)
hold on
plot(Mat_V_eq(:),Mat_U_eq(3,:))
xlabel('V (m/s)')
ylabel('Aileron (º)')
legend('Multi-body')

subplot(2,1,2)
hold on
plot(Mat_V_eq(:),Mat_U_eq(4,:))
xlabel('V (m/s)')
ylabel('Rudder (º)')




save Rotation_Multi_Body_crr.mat Mat_X_eq Mat_U_eq Mat_Y_eq Mat_V_eq