function visualizator_AB266(X)

% Adated from:https://www.mathworks.com/matlabcentral/fileexchange/5656-trajectory-and-attitude-plot-version-3
% Adapted by: Matheus Ribeiro Sampaio T-21
% matheus.ribeiro.aer@gmail.com
% All credits belong to the original author ( Valerio Scordamaglia).


%-------------------------------------------------
%  X vetor de estados padrao de AB-266
%   % X=[u
%    w
%    q
%    teta
%    h
%    x 
%{moveimento longitudinal 6 primeiras}
%{movimento latero direcional}
%    v
%    phi
%    p
%    r
%    psi
%    y]
%

%--------------------------------------------------------------------
% Configurações de tamanho
% So alterar aqui

scale_factor=10; % Tamanho do modelo do avião 

step=15; % Velocidade de reprodução

%Posicionamento da camera

% view=[ 0  -1  0]; % Plano XZ (longitudinal)
% view=[ 1  0  0]; % Plano YZ (rolamento)
% view=[ 0  0  1]; % Plano XY (guinada)
view=[ -6  -5  1]; % Isometrica


%-----------------------------------------------------------------------
% So mexer se souber
% trajetoria (m)
x=X(:,6);
y=X(:,12);
z=X(:,5);

% Angulos de euler (rad)

pitch=X(:,4)*pi/180;
yaw=X(:,11)*pi/180;
roll=-X(:,8)*pi/180;


selector='gripen';
scale_factor_proporcional=scale_factor/(max(x)+max(y)+max(z));% torna independente do tamanho dos eixos


[M]=trajectory3(x,y,z,pitch,roll,yaw,scale_factor_proporcional,step,selector,view);
