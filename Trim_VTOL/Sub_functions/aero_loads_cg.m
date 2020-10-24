function [F_aero_b,M_aero_O_b,Y_aero] = aero_loads_cg(X,U,aircraft)

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

% if u <2
%     F_aero_b = [0;0;0];
%     M_aero_O_b= [0;0;0];
%     Y_aero = [V
%     0
%     0
%     zeros(9,1)];
% 
% else
    
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

 de_deg= U(2,1); % 
 da_deg= U(3,1); % 
 dr_deg= U(4,1); %

%% Calculo coeficientes

D = calc_damp;

CL = calc_CL(alpha_rad,de_deg);
CD = calc_CD(CL);
CY = calc_CY(beta_rad,dr_deg);

Cl = calc_Cl(beta_rad,da_deg,dr_deg);
Cm = calc_Cm(alpha_rad,de_deg);
Cn = calc_Cn(beta_rad,da_deg,dr_deg);


% CXq  = D(1);
CYr = D(2);
CYp = D(3);
CLq = D(4);
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

% Stability axes derivative
CD = CD;
CY = CY + CYr*r_ndml + CYp*p_ndml;
CL = CL + CLq*q_ndml;

Cl = Cl + Clr*r_ndml + Clp*p_ndml;
Cm = Cm + Cmq*q_ndml;
Cn = Cn + Cnr*r_ndml + Cnp*p_ndml;

% Wind axes (Segundo doc do AVL)

CD_wind= CD*cos(beta_rad) + CY*sin(beta_rad);
CS_wind =  CY*cos(beta_rad) - CD*sin(beta_rad);
CL_wind =  CL;

Cl_wind =  Cl*cos(beta_rad) + Cm*sin(beta_rad);
Cm_wind =  Cm*cos(beta_rad) - Cl*sin(beta_rad);
Cn_wind =  Cn;



% Transform to Body axes
C_bw = DCM_original(2,alpha_rad)*DCM_original(3,-beta_rad);

Coeffs = -C_bw*[CD_wind;CS_wind; CL_wind];
% Coeffs = -1*[CD_wind;CS_wind; CL_wind];
CX = Coeffs(1);
CY = Coeffs(2);
CZ = Coeffs(3);

Coeffs = 1*[Cl_wind;Cm_wind; Cn_wind];
Cl_b = Coeffs(1);
Cm_b = Coeffs(2);
Cn_b = Coeffs(3);

% Dinamic pressure

rho=ISA(h); 
q_bar = 0.5*rho*V^2;

Xa = q_bar*S*CX;
Ya = q_bar*S*CY;
Za = q_bar*S*CZ;

La = q_bar*S*b*Cl_b;
Ma = q_bar*S*c*Cm_b -(aircraft.r_0(1)- 0.317)*Za;
Na = q_bar*S*b*Cn_b;


F_aero_b = [Xa; Ya; Za];

M_aero_O_b = [La; Ma; Na];            

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
    CS_wind % Coeficiente de força lateral
    CL];


end

function D = calc_damp
% global u v w V

D(1)= 1.445699;  % CXq

D(2)=  0.605196; % CYr
D(3)= -0.146996; % CYp
D(4)= 12.558624;  % CLq
D(5)= 0.164125;  % Clr
D(6)= -0.348803; % Clp
D(7)= -29.695559; % Cmq
D(8)=  -0.305432; % Cnr 
D(9)= 0.071957; % Cnp

% D= zeros(9,1);

end


function CD_calc = calc_CD(CL)


CD_02= 0.1324;
CD_01= -0.0498;
CD_00= 0.0251;

CD_00 = CD_00+ 0.03; % Parasite Drag

CD_calc = CD_00 + CD_01*CL + CD_02*CL^2;

end


function CY_calc = calc_CY(beta,d3)


CYb =  -0.642012;
 

CYd3 =  -0.003506;

CY_calc = (CYb*beta) + CYd3*d3;

end


function CL_calc = calc_CL(alpha,d2)

% if alpha < 16*pi/180
% CLa =   3.314395;
% CLd2 =  0.0064;
% CL_calc = CLa*alpha  + CLd2*d2 + 0.04;
% 
% elseif 16*pi/180<alpha<32*pi/180
% CLa =   3.314395;
% CLd2 =  0.0064;
% CL_calc = -CLa*alpha  + CLd2*d2 + 0.9656;
% 
% else
%     CL_calc=0.04;
% end

CLa =   3.314395;
CLd2 =  0.0064;
CL_calc = CLa*alpha  + CLd2*d2 + 0.04;


end


function Cm_calc = calc_Cm(alpha,d2)

alpha_deg=alpha*180/pi;

Cmd2 =  -0.013304; 


Cma = 1.4e-05*alpha_deg^3 + 0.0003*alpha_deg^2 - 0.02*alpha_deg + 0.0045;



Cm_calc = Cma*alpha + Cmd2*d2 -0.1;


end


function Cl_calc = calc_Cl(beta,d1,d3)

 
Clb =  -0.139921;

Cld1 =  -0.001941;
Cld3 =  -0.001049;


Cl_calc = Clb*beta + Cld1*d1 + Cld3*d3;



end


function Cn_calc = calc_Cn(beta,d1,d3)

Cnb =   0.261227;


Cnd1 =  -0.000222;
Cnd3 =   0.002147;


Cn_calc =  Cnb*beta + Cnd1*d1 + Cnd3*d3;


end



