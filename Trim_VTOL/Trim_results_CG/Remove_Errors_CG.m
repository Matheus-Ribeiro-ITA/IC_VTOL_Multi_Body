clear
clc


load trim_results_CG_multi_body.mat

indice_struct = 4;


figure
hold on
plot(Mat_X_eq{indice_struct}(13,:)*180/pi)
xlabel('índice')
ylabel('Sigma (º)')
legend('Multi-body')


indice_remover = [15,14];

for i=1:length(indice_remover)
    Mat_V_eq{indice_struct}(:,indice_remover(i)) = [];
    Mat_U_eq{indice_struct}(:,indice_remover(i)) = [];
    Mat_X_eq{indice_struct}(:,indice_remover(i)) = [];
    Mat_Y_eq{indice_struct}(:,indice_remover(i)) = [];
end



figure
subplot(2,1,1)
hold on
plot(Mat_V_eq{indice_struct}(:),Mat_U_eq{indice_struct}(1,:)*100)
xlabel('V (m/s)')
ylabel('Thrust (%)')
legend('Multi-body')

subplot(2,1,2)
hold on 
plot(Mat_V_eq{indice_struct}(:),Mat_U_eq{indice_struct}(2,:))
xlabel('V (m/s)')
ylabel('Elevator (º)')
axis([0 20 -20 20 ])

figure
subplot(2,1,1)
hold on
plot(Mat_V_eq{indice_struct}(:),Mat_X_eq{indice_struct}(13,:)*180/pi)
xlabel('V (m/s)')
ylabel('Sigma (º)')
legend('Multi-body')

subplot(2,1,2)
hold on
plot(Mat_V_eq{indice_struct}(:),Mat_Y_eq{indice_struct}(8,:))
xlabel('V (m/s)')
ylabel('alpha (º)')



save trim_results_CG_multi_body_crr.mat Mat_X_eq Mat_U_eq Mat_Y_eq Mat_V_eq