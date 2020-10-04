%% eVTOL Dynamics Modeling
% Author: Matheus Ribeiro T-21
clc
clear
close all

tic
%% --------------------------------------------------------------------------------
% Aircraft Parameters and Functions
% ---------------------------------------------------------------------
addpath ..\
addpath ..\Sub_functions
addpath ..\Trim_results_multi_body
addpath ..\Trimagem_multi_body
addpath ..\Aircraft_data

load aircraft

%% --------------------------------------------------------------------------------
%  --------------------------------------------------------------------------------
%  CG Variation
%  ---------------------------------------------------------------------
% --------------------------------------------------------------------------------

%% Trim calculation

i=1;

% First initial conditions

cg(1,1)= 0.29;
cg(2,1) = 0;        % PN=0.3 Catia = 0.3336
cg(3,1)= 0.0922;    % PN=    Catia = 0.0922

carga = 1;

% % Conserta o CG do corpo rigido
% aircraft = aircraft_gen(carga,cg);
% 
% % V_eq = 2; % m/s
% h_eq = 0;
% chi_eq = 0;
% gamma_eq = 0;
% phi_dot_eq = 0;
% theta_dot_eq = 0;
% psi_dot_eq = 0;
% sigma_eq = 0;


% beta_eq = 0;

%%

cg_x =[0.27,0.29,0.31,0.33];


for i=1:length(cg_x)
     cg(1,1) = cg_x(i);
    [Mat_X_eq{i},Mat_U_eq{i}, Mat_Y_eq{i}, Mat_V_eq{i}] = multi_body_CG(cg);
end
save ..\Trim_results_CG/trim_results_CG_multi_body.mat Mat_X_eq Mat_U_eq Mat_Y_eq Mat_V_eq

for i=1:length(cg_x)
     cg(1,1) = cg_x(i);
    [Mat_X_eq{i},Mat_U_eq{i}, Mat_Y_eq{i}, Mat_V_eq{i}] = uni_body_CG(cg);
end
save ..\Trim_results_CG/trim_results_CG_uni_body.mat Mat_X_eq Mat_U_eq Mat_Y_eq Mat_V_eq


%%
time_opt= toc

save time.mat time_opt