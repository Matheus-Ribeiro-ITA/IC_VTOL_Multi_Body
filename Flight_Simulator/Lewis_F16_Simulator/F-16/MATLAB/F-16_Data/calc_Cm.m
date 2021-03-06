function Cm_calc = calc_Cm(alpha,el) % Cm0

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