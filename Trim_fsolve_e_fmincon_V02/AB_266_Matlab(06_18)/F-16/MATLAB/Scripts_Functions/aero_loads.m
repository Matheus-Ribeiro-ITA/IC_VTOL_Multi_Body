function [F_aero_b,M_aero_O_b,Y_aero] = aero_loads(X,U,aircraft)

% INCOMPLETE FILE! IT WILL BE COMPLETED IN THE CLASS quase

u = X(1);
w = X(2);
v = X(7);

V = sqrt(u^2+v^2+w^2);
alpha_rad= atan(w/u);
alpha_deg= alpha_rad*180/pi;
beta_rad= asin(v/V);
beta_deg= beta_rad*180/pi;

de_deg= U(2);
da_deg= U(3);
dr_deg= U(4);

D = calc_damp(alpha_deg);

CX = calc_CX(alpha_deg,de_deg);
CY = calc_CY(beta_deg,da_deg,dr_deg);
CZ = calc_CZ(alpha_deg,beta_deg,de_deg);

Cl = calc_Cl(alpha_deg,beta_deg);
Clda = calc_dClda(alpha_deg,beta_deg);
Cldr = calc_dCldr(alpha_deg,beta_deg);

Cm = calc_Cm(alpha_deg,de_deg);

Cn = calc_Cn(alpha_deg,beta_deg);
Cnda = calc_dCnda(alpha_deg,beta_deg);
Cndr = calc_dCndr(alpha_deg,beta_deg);

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

p_rad_s = p_deg_s*pi/180;
q_rad_s = q_deg_s*pi/180;
r_rad_s = r_deg_s*pi/180;
p_ndml = p_rad_s*b/(2*V);
q_ndml = q_rad_s*c/(2*V);
r_ndml = r_rad_s*b/(2*V);

CX = CX + CXq*q_ndml;
CY = CY + CYr*r_ndml + CYp*p_ndml;
CZ = CZ + CZq*q_ndml;

Cl = Cl + Clda*(da_deg/20.0) + Cldr*(dr_deg/30.0) + ...
    Clr*r_ndml + Clp*p_ndml;
Cm = Cm + Cmq*q_ndml;
Cn = Cn + Cnda*(da_deg/20.0) + Cndr*(dr_deg/30.0) + ...
    Cnr*r_ndml + Cnp*p_ndml;


rho=ISA(h); % So usar atmoISA ja pronta do Matlab
q_bar = 0.5*rho*V^2;

Xa = q_bar*S*CX;
Ya = q_bar*S*CY;
Za = q_bar*S*CZ;

La = q_bar*S*b*Cl;
Ma = q_bar*S*c*Cm;
Na = q_bar*S*b*Cn;

F_aero_b = [Xa; Ya; Za];

M_aero_O_b = [La; Ma; Na];

C_bw = DCM(2,alpha_rad)*DCM(3,-beta_rad);
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

function D = calc_damp(alpha)

A = [-.267 -.110 .308 1.34 2.08 2.91 2.76 2.05 1.50 1.49 1.83 1.21
    .882 .852 .876 .958 .962 .974 .819 .483 .590 1.21 -.493 -1.04
    -.108 -.108 -.188 .110 .258 .226 .344 .362 .611 .529 .298 -2.27
    -8.80 -25.8 -28.9 -31.4 -31.2 -30.7 -27.7 -28.2 -29.0 -29.8 -38.3 -35.3
    -.126 -.026 .063 .113 .208 .230 .319 .437 .680 .100 .447 -.330
    -.360 -.359 -.443 -.420 -.383 -.375 -.329 -.294 -.230 -.210 -.120 -.100
    -7.21 -.540 -5.23 -5.26 -6.11 -6.64 -5.69 -6.00 -6.20 -6.40 -6.60 -6.00
    -.380 -.363 -.378 -.386 -.370 -.453 -.550 -.582 -.595 -.637 -1.02 -.840
    .061 .052 .052 -.012 -.013 -.024 .050 .150 .130 .158 .240 .150].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

D = A(k+3,:) + abs(da)* (A(l+3,:)-A(k+3,:));
D = D.';

% CXq = D(1);
% CYr = D(2);
% CYp = D(3);
% CZq = D(4);
% Clr = D(5);
% Clp = D(6);
% Cmq = D(7);
% Cnr = D(8);
% Cnp = D(9);

end


function CX_calc = calc_CX(alpha,el)

A = [-.099 -.081 -.081 -.063 -.025 .044 .097 .113 .145 .167 .174 .166 
    -.048 -.038 -.040 -.021 .016 .083 .127 .137 .162 .177 .179 .167
    -.022 -.020 -.021 -.004 .032 .094 .128 .130 .154 .161 .155 .138
    -.040 -.038 -.039 -.025 .006 .062 .087 .085 .100 .110 .104 .091
    -.083 -.073 -.076 -.072 -.046 .012 .024 .025 .043 .053 .047 .040].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = el/12.0;

m = fix(s);

if m<=-2, m = -1; end
if m>=2, m = 1; end

de = s-m;

n = m+fix(1.1*sign(de));

t = A(k+3,m+3);
u = A(k+3,n+3);
v = t+abs(da)*(A(l+3,m+3)-t);
w = u+abs(da)*(A(l+3,n+3)-u);

CX_calc = v+(w-v)*abs(de);

end


function CY_calc = calc_CY(beta,ail,rdr)

CY_calc = -.02*beta+.021*(ail/20.0)+.086*(rdr/30.0);

end


function CZ_calc = calc_CZ(alpha,beta,el)

A = [.770 .241 -.100 -.416 -.731 -1.053 -1.366 -1.646 -1.917 -2.120 -2.248 -2.229].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = A(k+3)+abs(da)*(A(l+3)-A(k+3));

CZ_calc = s*(1-(beta/57.3)^2)-.19*(el/25.0);

end


function Cm_calc = calc_Cm(alpha,el)

A = [.205 .168 .186 .196 .213 .251 .245 .238 .252 .231 .198 .192
    .081 .077 .107 .110 .110 .141 .127 .119 .133 .108 .081 .093
    -.046 -.020 -.009 -.005 -.006 .010 .006 -.001 .014 .000 -.013 .032
    -.174 -.145 -.121 -.127 -.129 -.102 -.097 -.113 -.087 -.084 -.069 -.006
    -.259 -.202 -.184 -.193 -.199 -.150 -.160 -.167 -.104 -.076 -.041 -.005].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = el/12.0;

m = fix(s);

if m<=-2, m = -1; end
if m>=2, m = 1; end

de = s-m;

n = m+fix(1.1*sign(de));

t = A(k+3,m+3);
u = A(k+3,n+3);
v = t+abs(da)*(A(l+3,m+3)-t);
w = u+abs(da)*(A(l+3,n+3)-u);

Cm_calc = v+(w-v)*abs(de);

end


function Cl_calc = calc_Cl(alpha,beta)

% 3. ed.:
% Correction implemented by Juliano Paulino and Guilherme Barbosa:
A = [zeros(1,12)
    -.001 -.004 -.008 -.012 -.016 -.019 -.020 -.020 -.015 -.008 -.013 -.015
    -.003 -.009 -.017 -.024 -.030 -.034 -.040 -.037 -.016 -.002 -.010 -.019
    -.001 -.010 -.020 -.030 -.039 -.044 -.050 -.049 -.023 -.006 -.014 -.027
    .000 -.010 -.022 -.034 -.047 -.046 -.059 -.061 -.033 -.036 -.035 -.035
    .007 -.010 -.023 -.034 -.049 -.046 -.068 -.071 -.060 -.058 -.062 -.059
    .009 -.011 -.023 -.037 -.050 -.047 -.074 -.079 -.091 -.076 -.077 -.076].';
% 2. ed.:
% Correction implemented by Juliano Paulino and Guilherme Barbosa:
% A = [zeros(1,12)
%     -.001 -.004 -.008 -.012 -.016 -.022 -.022 -.021 -.015 -.008 -.013 -.015
%     -.003 -.009 -.017 -.024 -.030 -.041 -.045 -.040 -.016 -.002 -.010 -.019
%     -.001 -.010 -.020 -.030 -.039 -.054 -.057 -.054 -.023 -.006 -.014 -.027
%     .000 -.010 -.022 -.034 -.047 -.060 -.069 -.067 -.033 -.036 -.035 -.035
%     .007 -.010 -.023 -.034 -.049 -.063 -.081 -.079 -.060 -.058 -.062 -.059
%     .009 -.011 -.023 -.037 -.050 -.068 -.089 -.088 -.091 -.076 -.077 -.076].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = 0.2*abs(beta);

m = fix(s);

if m<=0, m = 1; end
if m>=6, m = 5; end

db = s-m;

n = m+fix(1.1*sign(db));

t = A(k+3,m+1);
u = A(k+3,n+1);
v = t+abs(da)*(A(l+3,m+1)-t);
w = u+abs(da)*(A(l+3,n+1)-u);

dum = v+(w-v)*abs(db);

Cl_calc = dum*sign(beta);

end


function Cn_calc = calc_Cn(alpha,beta)

A = [zeros(1,12)
    .018 .019 .018 .019 .019 .018 .013 .007 .004 -.014 -.017 -.033
    .038 .042 .042 .042 .043 .039 .030 .017 .004 -.035 -.047 -.057
    .056 .057 .059 .058 .058 .053 .032 .012 .002 -.046 -.071 -.073
    .064 .077 .076 .074 .073 .057 .029 .007 .012 -.034 -.065 -.041
    .074 .086 .093 .089 .080 .062 .049 .022 .028 -.012 -.002 -.013
    .079 .090 .106 .106 .096 .080 .068 .030 .064 .015 .011 -.001].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = 0.2*abs(beta);

m = fix(s);

if m<=0, m = 1; end
if m>=6, m = 5; end

db = s-m;

n = m+fix(1.1*sign(db));

t = A(k+3,m+1);
u = A(k+3,n+1);
v = t+abs(da)*(A(l+3,m+1)-t);
w = u+abs(da)*(A(l+3,n+1)-u);

dum = v+(w-v)*abs(db);

Cn_calc = dum*sign(beta);

end


function dClda_calc = calc_dClda(alpha,beta)

A = [-.041 -.052 -.053 -.056 -.050 -.056 -.082 -.059 -.042 -.038 -.027 -.017
    -.041 -.053 -.053 -.053 -.050 -.051 -.066 -.043 -.038 -.027 -.023 -.016
    -.042 -.053 -.052 -.051 -.049 -.049 -.043 -.035 -.026 -.016 -.018 -.014
    -.040 -.052 -.051 -.052 -.048 -.048 -.042 -.037 -.031 -.026 -.017 -.012
    -.043 -.049 -.048 -.049 -.043 -.042 -.042 -.036 -.025 -.021 -.016 -.011
    -.044 -.048 -.048 -.047 -.042 -.041 -.020 -.028 -.013 -.014 -.011 -.010
    -.043 -.049 -.047 -.045 -.042 -.037 -.003 -.013 -.010 -.003 -.007 -.008].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = 0.1*beta;

m = fix(s);

if m<=-3, m = -2; end
if m>= 3, m = 2; end

db = s-m;

n = m+fix(1.1*sign(db));

t = A(k+3,m+4);
u = A(k+3,n+4);
v = t+abs(da)*(A(l+3,m+4)-t);
w = u+abs(da)*(A(l+3,n+4)-u);

dClda_calc = v+(w-v)*abs(db);

end


function dCldr_calc = calc_dCldr(alpha,beta)

A = [.005 .017 .014 .010 -.005 .009 .019 .005 -.000 -.005 -.011 .008
    .007 .016 .014 .014 .013 .009 .012 .005 .000 .004 .009 .007
    .013 .013 .011 .012 .011 .009 .008 .005 -.002 .005 .003 .005
    .018 .015 .015 .014 .014 .014 .014 .015 .013 .011 .006 .001
    .015 .014 .013 .013 .012 .011 .011 .010 .008 .008 .007 .003
    .021 .011 .010 .011 .010 .009 .008 .010 .006 .005 .000 .001
    .023 .010 .011 .011 .011 .010 .008 .010 .006 .014 .020 .000].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = 0.1*beta;

m = fix(s);

if m<=-3, m = -2; end
if m>= 3, m = 2; end

db = s-m;

n = m+fix(1.1*sign(db));

t = A(k+3,m+4);
u = A(k+3,n+4);
v = t+abs(da)*(A(l+3,m+4)-t);
w = u+abs(da)*(A(l+3,n+4)-u);

dCldr_calc = v+(w-v)*abs(db);

end


function dCnda_calc = calc_dCnda(alpha,beta)

A = [.001 -.027 -.017 -.013 -.012 -.016 .001 .017 .011 .017 .008 .016
    .002 -.014 -.016 -.016 -.014 -.019 -.021 .002 .012 .015 .015 .011
    -.006 -.008 -.006 -.006 -.005 -.008 -.005 .007 .004 .007 .006 .006
    -.011 -.011 -.010 -.009 -.008 -.006 .000 .004 .007 .010 .004 .010
    -.015 -.015 -.014 -.012 -.011 -.008 -.002 .002 .006 .012 .011 .011
    -.024 -.010 -.004 -.002 -.001 .003 .014 .006 -.001 .004 .004 .006
    -.022 .002 -.003 -.005 -.003 -.001 -.009 -.009 -.001 .003 -.002 .001].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = 0.1*beta;

m = fix(s);

if m<=-3, m = -2; end
if m>= 3, m = 2; end

db = s-m;

n = m+fix(1.1*sign(db));

t = A(k+3,m+4);
u = A(k+3,n+4);
v = t+abs(da)*(A(l+3,m+4)-t);
w = u+abs(da)*(A(l+3,n+4)-u);

dCnda_calc = v+(w-v)*abs(db);

end


function dCndr_calc = calc_dCndr(alpha,beta)

A = [-.018 -.052 -.052 -.052 -.054 -.049 -.059 -.051 -.030 -.037 -.026 -.013
    -.028 -.051 -.043 -.046 -.045 -.049 -.057 -.052 -.030 -.033 -.030 -.008
    -.037 -.041 -.038 -.040 -.040 -.038 -.037 -.030 -.027 -.024 -.019 -.013
    -.048 -.045 -.045 -.045 -.044 -.045 -.047 -.048 -.049 -.045 -.033 -.016
    -.043 -.044 -.041 -.041 -.040 -.038 -.034 -.035 -.035 -.029 -.022 -.009
    -.052 -.034 -.036 -.036 -.035 -.028 -.024 -.023 -.020 -.016 -.010 -.014
    -.062 -.034 -.027 -.028 -.027 -.027 -.023 -.023 -.019 -.009 -.025 -.010].';

s = 0.2*alpha;

k = fix(s);

if k<=-2, k = -1; end
if k>=9, k = 8; end

da = s-k;

l = k+fix(1.1*sign(da));

s = 0.1*beta;

m = fix(s);

if m<=-3, m = -2; end
if m>= 3, m = 2; end

db = s-m;

n = m+fix(1.1*sign(db));

t = A(k+3,m+4);
u = A(k+3,n+4);
v = t+abs(da)*(A(l+3,m+4)-t);
w = u+abs(da)*(A(l+3,n+4)-u);

dCndr_calc = v+(w-v)*abs(db);

end

