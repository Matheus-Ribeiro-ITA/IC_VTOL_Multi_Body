% International Standard Atmosphere (ISA) in SI units
% Input : h = altitute (m)
% Outputs : T = temperature (K), p = pressure (N/mˆ2), rho = density (kg/mˆ3), a = sound speed (m/s)

% So usar atmoISA ja pronta do Matlab
function [rho,T,p,a]=ISA(h)

h = h/1000; % m -> km

h1 = 11;
h2 = 20;
h3 = 32;

L0 = -6.5e-3; 
L2 = 1e-3; 

g0 = 9.80665;
m0 = 28.96442;
R0 = 8314.32; 
R = R0/m0;

T0 = 288.15;
p0 = 1.01325e5;
rho0 = 1.2250;

T1 = T0+L0*h1*1e3;
p1 = p0*(T1/T0)^(-g0/(R*L0)); 
rho1 = rho0*(T1/T0)^(-(1+g0/(R*L0))); 

T2 = T1;
p2 = p1*exp(-g0/(R*T2)*(h2-h1)*1e3);
rho2 = rho1*exp(-g0/(R*T2)*(h2-h1)*1e3);

if h<=h1
	% Troposphere:
	T = T0+L0*h*1e3;
	p = p0*(T/T0)^(-g0/(R*L0));
	rho = rho0*(T/T0)^(-(1+g0/(R*L0)));
elseif h<=h2
	% Tropopause and low stratosphere:
	T = T1;
	p = p1*exp(-g0/(R*T)*(h-h1)*1e3);
	rho=rho1*exp(-g0/(R*T)*(h-h1)*1e3);
elseif h<=h3
	% Stratosphere:
	T = T2+L2*(h-h2)*1e3;
	p = p2*(T/T2)^(-g0/(R*L2));
	rho = rho2*(T/T2)^(-(1+g0/(R*L2)));
end

gama = 1.4;

a = sqrt(gama*R*T);