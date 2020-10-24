
%%
figure('Name','Longitudinal state variables')

subplot(231)
plot(T,X(:,1))
xlabel('t [s]')
ylabel('u [m/s]')

subplot(232)
plot(T,X(:,2))
xlabel('t [s]')
ylabel('w [m/s]')

subplot(233)
plot(T,X(:,3))
xlabel('t [s]')
ylabel('q [deg/s]')

subplot(234)
plot(T,X(:,4))
xlabel('t [s]')
ylabel('\theta [deg]')

subplot(235)
plot(T,X(:,5))
xlabel('t [s]')
ylabel('h [m]')

subplot(236)
plot(T,X(:,6))
xlabel('t [s]')
ylabel('x [m]')

%% Latero dire
figure('Name','Lateral-directional state variables')

subplot(231)
plot(T,X(:,7))
xlabel('t [s]')
ylabel('v [m/s]')

subplot(232)
plot(T,X(:,8))
xlabel('t [s]')
ylabel('\phi [deg]')

subplot(233)
plot(T,X(:,9))
xlabel('t [s]')
ylabel('p [deg/s]')

subplot(234)
plot(T,X(:,10))
xlabel('t [s]')
ylabel('r [deg/s]')

subplot(235)
plot(T,X(:,11))
xlabel('t [s]')
ylabel('\psi [deg]')

subplot(236)
plot(T,X(:,12))
xlabel('t [s]')
ylabel('y [m]')

%% Entradas do piloto
figure('Name','Dutch-roll polar plots')

subplot(131)
plot(X(:,11),X(:,8))
hold on
plot(X(1,11),X(1,8),'*')
xlabel('\psi [deg]')
ylabel('\phi [deg]')

subplot(132)
plot(Y(:,3),X(:,8))
hold on
plot(Y(1,3),X(1,8),'*')
xlabel('\beta [deg]')
ylabel('\phi [deg]')

subplot(133)
plot(X(:,9),X(:,10))
hold on
plot(X(1,9),X(1,10),'*')
xlabel('p [deg/s]')
ylabel('r [deg/s]')

%% Trajetoria
figure('Name','Flight path')
plot3(X(:,12),X(:,6),X(:,5))
xlabel('y [m]')
ylabel('x [m]')
zlabel('h [m]')
grid on
axis equal

%% Variaveis de controle
figure('Name','Control variables')

subplot(221)
plot(T,U(:,1)*100)
xlabel('t [s]')
ylabel('\delta_t [%]')

if not(exist('Ucom','var'))
    subplot(222)
    plot(T,U(:,2))
    xlabel('t [s]')
    ylabel('\delta_e [deg]')

    subplot(223)
    plot(T,U(:,3))
    xlabel('t [s]')
    ylabel('\delta_a [deg]')

    subplot(224)
    plot(T,U(:,4))
    xlabel('t [s]')
    ylabel('\delta_r [deg]')
else
    subplot(222)
    plot(T,U(:,2))
    hold all
    plot(T,Ucom(:,2))
    xlabel('t [s]')
    ylabel('\delta_e [deg]')

    subplot(223)
    plot(T,U(:,3))
    hold all
    plot(T,Ucom(:,3))
    xlabel('t [s]')
    ylabel('\delta_a [deg]')

    subplot(224)
    plot(T,U(:,4))
    hold all
    plot(T,Ucom(:,4))
    xlabel('t [s]')
    ylabel('\delta_r [deg]')
    
    legend('True','Commanded','Location','Best')
end

%% Atuadores
figure('Name','Some outputs')

subplot(231)
plot(T,Y(:,1))
xlabel('t [s]')
ylabel('V [m/s]')

subplot(232)
plot(T,Y(:,2))
xlabel('t [s]')
ylabel('\alpha [deg]')

subplot(233)
plot(T,Y(:,3))
xlabel('t [s]')
ylabel('\beta [deg]')

subplot(234)
plot(T,X(:,13))
xlabel('t [s]')
ylabel('Power [%]')

subplot(235)
plot(T,Y(:,end-1))
xlabel('t [s]')
ylabel('n_{y,P,b}')

subplot(236)
plot(T,Y(:,end))
xlabel('t [s]')
ylabel('n_{z,P,b}')

%%
figure('Name','Aerodynamic coefficients')

subplot(231)
plot(T,Y(:,10))
xlabel('t [s]')
ylabel('C_D')

subplot(232)
plot(T,Y(:,11))
xlabel('t [s]')
ylabel('C_Y')

subplot(233)
plot(T,Y(:,12))
xlabel('t [s]')
ylabel('C_L')

subplot(234)
plot(T,Y(:,7))
xlabel('t [s]')
ylabel('C_l')

subplot(235)
plot(T,Y(:,8))
xlabel('t [s]')
ylabel('C_m')

subplot(236)
plot(T,Y(:,9))
xlabel('t [s]')
ylabel('C_n')

%% Vento
if exist('Vw','var')
    figure('Name','Wind components in LVLH system')

    subplot(311)
    plot(T,Vw(:,1))
    xlabel('t [s]')
    ylabel('V_{w,x} [m/s]')

    subplot(312)
    plot(T,Vw(:,2))
    xlabel('t [s]')
    ylabel('V_{w,y} [m/s]')

    subplot(313)
    plot(T,Vw(:,3))
    xlabel('t [s]')
    ylabel('V_{w,z} [m/s]')
end
