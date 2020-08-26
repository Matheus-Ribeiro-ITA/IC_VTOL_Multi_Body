function visualizator_VTOL(X)

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



step=1; % Velocidade de reprodução

%Posicionamento da camera

% view=[ 0  -1  0]; % Plano XZ (longitudinal)
% view=[ 1  0  0]; % Plano YZ (rolamento)
% view=[ 0  0  1]; % Plano XY (guinada)
view=[ -6  -5  1]; % Isometrica


%-----------------------------------------------------------------------
% So mexer se souber
% trajetoria (m)
x=X(:,1);
y=X(:,2);
z=-X(:,3);

% Angulos de euler (rad)

pitch=X(:,11);
yaw=X(:,12);
roll=-X(:,10);


selector='gripen';
scale_factor_proporcional=scale_factor/(max(x)+max(y)+max(z));% torna independente do tamanho dos eixos


[M]=trajectory3(x,y,z,pitch,roll,yaw,scale_factor_proporcional,step,selector,view);
