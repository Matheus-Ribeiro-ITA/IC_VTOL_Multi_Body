% Aircraft Struct Generator

close all
clear
clc

cg(1,1)= 0.3336;
cg(2,1) = 0;        % PN=0.3 Catia = 0.3336
cg(3,1)= 0.0922;    % PN=    Catia = 0.0922

carga = 1;


aircraft = aircraft_gen(carga,cg);
save aircraft.mat aircraft