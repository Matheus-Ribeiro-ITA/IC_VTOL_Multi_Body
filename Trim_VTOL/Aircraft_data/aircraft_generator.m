% -------------------------------------------------------
% Aircraft Struct Generator
% -------------------------------------------------------
% Author: Matheus Ribeiro T-21
% gmail: matehus.ribeiro.aer@gmail.com
% Version: 0.0.1 
% Description: Generates the .mat aircraft arquive with the correct 
%              Cg correction.
% -------------------------------------------------------

close all
clear
clc

cg(1,1)= 0.3336;
cg(2,1) = 0;        % PN=0.3 Catia = 0.3336
cg(3,1)= 0.0922;    % PN=    Catia = 0.0922

carga = 1;


aircraft = aircraft_gen(carga,cg);
save aircraft.mat aircraft