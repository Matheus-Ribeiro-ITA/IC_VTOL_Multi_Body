function [F_prop_b,M_prop_O_b,Y_prop] = prop_loads(X,U,aircraft)

global m2ft lbf2N

power = X(13);

u = X(1);
w= X(2);
v=X(7);
h=X(5);
p_deg_s=X(9);
r_deg_s=X(10);
q_deg_s=X(3);


V = sqrt(u^2+v^2+w^2);
% Thrust:
[~,~,~,a]= ISA(h);
M = V/a;

h_ft = h*m2ft;
T = thrust(power,h_ft,M); % lbf
T = T*lbf2N;

% Power time derivative:

throttle = U(1);
cpower = tgear(throttle);
pdot_calc = pdot(power,cpower);

% No sistema do corpo:

F_prop_b = [T; 0; 0];

p_rad_s= p_deg_s*pi/180;
q_rad_s= q_deg_s*pi/180;
r_rad_s= r_deg_s*pi/180;

omega_b = [p_rad_s; q_rad_s; r_rad_s];
hex = aircraft.prop.hex;
M_prop_O_b = -skew(omega_b)*[hex; 0; 0];

Y_prop = [pdot_calc
    T
    cpower
    power];

end

function cpow = tgear(throttle)

if throttle<=0.77
    cpow = 64.94*throttle;
else
    cpow = 217.38*throttle-117.38;
end

end

function pdot_calc = pdot(p3,p1)

if p1>=50
    if p3>=50
        T = 5;
        p2 = p1;
    else
        p2 = 60;
        T = rtau(p2-p3);
    end
else
    if p3>=50
        T = 5;
        p2 = 40;
    else
        p2 = p1;
        T = rtau(p2-p3);
    end
end
pdot_calc = T*(p2-p3);

end

function rtau_calc = rtau(dp)

if dp<=25
    rtau_calc = 1;
elseif dp>=50
    rtau_calc = 0.1;
else
    rtau_calc = 1.9-0.036*dp;
end

end

function T = thrust(pow,alt,rmach)

A = [1060.0 670.0 880.0 1140.0 1500.0 1860.0
    635.0 425.0 690.0 1010.0 1330.0 1700.0
    60.0 25.0 345.0 755.0 1130.0 1525.0
    -1020.0 -710.0 -300.0 350.0 910.0 1360.0
    -2700.0 -1900.0 -1300.0 -247.0 600.0 1100.0
    -3600.0 -1400.0 -595.0 -342.0 -200.0 700.0].';

B = [12680.0 9150.0 6200.0 3950.0 2450.0 1400.0 
    12680.0 9150.0 6313.0 4040.0 2470.0 1400.0 
    12610.0 9312.0 6610.0 4290.0 2600.0 1560.0 
    12640.0 9839.0 7090.0 4660.0 2840.0 1660.0 
    12390.0 10176.0 7750.0 5320.0 3250.0 1930.0
    11680.0 9848.0 8050.0 6100.0 3800.0 2310.0].';

C = [20000.0 15000.0 10800.0 7000.0 4000.0 2500.0
    21420.0 15700.0 11225.0 7323.0 4435.0 2600.0
    22700.0 16860.0 12250.0 8154.0 5000.0 2835.0
    24240.0 18910.0 13760.0 9285.0 5700.0 3215.0
    26070.0 21075.0 15975.0 11115.0 6860.0 3950.0
    28886.0 23319.0 18300.0 13484.0 8642.0 5057.0].';

h = 0.0001*alt;
i = fix(h);

if i>=5, i = 4; end

dh = h-i;

rm = 5*rmach;

m = fix(rm);

if m>=5, m = 4; end

dm = rm-m;

cdh = 1.0-dh;

s = B(i+1,m+1)*cdh + B(i+1+1,m+1)*dh;

t = B(i+1,m+1+1)*cdh + B(i+1+1,m+1+1)*dh;

tmil = s+(t-s)*dm;

if pow<50
    s = A(i+1,m+1)*cdh + A(i+1+1,m+1)*dh;
    t = A(i+1,m+1+1)*cdh + A(i+1+1,m+1+1)*dh;
    tidl = s+(t-s)*dm;
    T = tidl+(tmil-tidl)*pow*0.02;
else
    s = C(i+1,m+1)*cdh + C(i+1+1,m+1)*dh;
    t = C(i+1,m+1+1)*cdh + C(i+1+1,m+1+1)*dh;
    tmax = s+(t-s)*dm;
    T = tmil+(tmax-tmil)*(pow-50)*0.02;
end

end