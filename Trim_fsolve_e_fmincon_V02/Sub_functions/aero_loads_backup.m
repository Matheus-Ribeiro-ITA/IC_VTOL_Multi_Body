function [F_aero_b,M_aero_O_b,Y_aero] = aero_loads(X,U,aircraft)

%Colcar U no começo de novo

global u v w V
u = X(4);
w = X(6);
v = X(5);

q=X(11);
p=X(10);
r=X(12);

h=X(3);

V = sqrt(u^2+v^2+w^2);

if V == 0
    V=0.001;
end

if u <2
    F_aero_b = [0;0;0];
    M_aero_O_b= [0;0;0];
    Y_aero = [V
    0
    0
    zeros(9,1)];

else
    
alpha_rad= atan(w/u);
alpha_deg= alpha_rad*180/pi;
beta_rad= asin(v/V);
beta_deg= beta_rad*180/pi;

if isnan(alpha_rad) == 1
    alpha_rad=0;
end

if isnan(beta_rad) == 1
    beta_rad=0;
end

%  de_deg= U(2);
%  da_deg= U(3);
%  dr_deg= U(4);

 de_deg= U(2,1); % 
 da_deg= U(3,1); % 
 dr_deg= U(4,1); %

%% Calculo coeficientes

D = calc_damp(alpha_deg);

CX = calc_CX(alpha_deg,de_deg,da_deg,dr_deg);
CY = calc_CY(beta_deg,de_deg,da_deg,dr_deg);
CZ = calc_CZ(alpha_deg,beta_deg,de_deg,da_deg,dr_deg);

Cl = calc_Cl(alpha_deg,beta_deg,de_deg,da_deg,dr_deg);
% Clda = calc_dClda(alpha_deg,beta_deg);
% Cldr = calc_dCldr(alpha_deg,beta_deg);

Cm = calc_Cm(alpha_deg,de_deg,da_deg,dr_deg);

Cn = calc_Cn(alpha_deg,beta_deg,de_deg,da_deg,dr_deg);
% Cnda = calc_dCnda(alpha_deg,beta_deg);
% Cndr = calc_dCndr(alpha_deg,beta_deg);

CXq = D(1);
CYr = D(2);
CYp = D(3);
CZq = D(4);
Clr = D(5);
Clp = D(6);
Cmq = D(7);
Cnr = D(8);
Cnp = D(9);

b = aircraft.geom.b;
c = aircraft.geom.c;
S = aircraft.geom.S;



p_rad_s = p;
q_rad_s = q;
r_rad_s = r;

p_ndml = p_rad_s*b/(2*V);
q_ndml = q_rad_s*c/(2*V);
r_ndml = r_rad_s*b/(2*V);

CX = CX + CXq*q_ndml;
CY = CY + CYr*r_ndml + CYp*p_ndml;
CZ = CZ + CZq*q_ndml;

Cl = Cl + Clr*r_ndml + Clp*p_ndml;
Cm = Cm + Cmq*q_ndml;
Cn = Cn; 

%Cnda*(da_deg/20.0) + Cndr*(dr_deg/30.0) + ...
%    Cnr*r_ndml + Cnp*p_ndml;

rho=ISA(h); % So usar atmoISA ja pronta do Matlab
q_bar = 0.5*rho*V^2;

Xa = q_bar*S*CX;
Ya = q_bar*S*CY;
Za = q_bar*S*CZ;

La = q_bar*S*b*Cl;
Ma = q_bar*S*c*Cm - aircraft.r_0(1)*Za;
Na = q_bar*S*b*Cn;

F_aero_b = [Xa; Ya; Za];

M_aero_O_b = [La; Ma; Na];

C_bw = DCM_original(2,alpha_rad)*DCM_original(3,-alpha_rad);
C_wb = C_bw.';
Coeffs = -C_wb*[CX;CY; CZ];
CD = Coeffs(1);
CS = Coeffs(2);
CL = Coeffs(3);
            

Y_aero = [V
    alpha_deg
    beta_deg
    CX
    CY
    CZ
    Cl
    Cm
    Cn
    CD
    CS % Coeficiente de força lateral
    CL];

end

end

function D = calc_damp(alpha)
% global u v w V

D(1)= 1.445699;  % CXq
D(2)=  0.365313; % CYr
D(3)= -0.125392; % CYp
D(4)= -12.2444;  % CZq
D(5)= 0.180254;  % Clr
D(6)= -0.338014; % Clp
D(7)= -29.89;    % Cmq
D(8)=  -0.212280; % Cnr 
D(9)= 0.012317; % Cnp

% D= zeros(9,1);

end


function CX_calc = calc_CX(alpha,d2,d1,d3)
global u v w V
% Teste para alpha beta e d's zeros

CXu =  -0.043894;
CXv =   0; 
% CXw =   0.173260;
CXw =   0;

CXd1 =   0;
CXd2 =   0.000120; 
CXd3 =   0;

CX_calc = (CXu*u/V + CXv*v/V + CXw*w/V) + CXd1*d1 + CXd2*d2 + CXd3*d3;

end


function CY_calc = calc_CY(beta,d2,d1,d3)
global u v w V
% CY_calc = -.02*beta+.021*(ail/20.0)+.086*(rdr/30.0);

CYu =  0;
CYv =  -0.642012;
CYw =  0;


% CYd1 =   0.000260; 
CYd1 =   0;
CYd2 =   0;  
CYd3 =  -0.003506;

CY_calc = (CYu*u/V + CYv*v/V + CYw*w/V) + CYd1*d1 + CYd2*d2 + CYd3*d3;

end


function CZ_calc = calc_CZ(alpha,beta,d2,d1,d3)
global u v w V
CZu =  -0.120444;
CZv =   0.000000;
CZw =  -3.337404;


CZd1 =  -0.000000;
CZd2 =  -0.006569;
CZd3 =  -0.000000;

CZ_calc = (CZu*u/V + CZv*v/V + CZw*w/V) + CZd1*d1 + CZd2*d2 + CZd3*d3;


end


function Cm_calc = calc_Cm(alpha,d2,d1,d3)
global u v w V

Cmu =  -0.383564;    
Cmv =   0.000000;   
Cmw =  -4.000180;


Cmd1 =  0;
Cmd2 =  -0.021187;
Cmd3 =   0;

Cm_calc = (Cmu*u/V + Cmv*v/V + Cmw*w/V) + Cmd1*d1 + Cmd2*d2 + Cmd3*d3;


end


function Cl_calc = calc_Cl(alpha,beta,d2,d1,d3)
global u v w V
 
Clu =  -0.000000;
Clv =  -0.139921;
Clw =  -0.000000;

Cld1 =  -0.001941;
Cld2 =  -0.000000;
Cld3 =  -0.001049;


Cl_calc = (Clu*u/V + Clv*v/V + Clw*w/V) + Cld1*d1 + Cld2*d2 + Cld3*d3;



end


function Cn_calc = calc_Cn(alpha,beta,d2,d1,d3)
global u v w V
Cnu =   0.000000;
Cnv =   0.261227;
Cnw =   0.000000;

Cnd1 =  -0.000222;
Cnd2 =   0.000000;
Cnd3 =   0.002147;


Cn_calc = (Cnu*u/V + Cnv*v/V + Cnw*w/V) + Cnd1*d1 + Cnd2*d2 + Cnd3*d3;


end



