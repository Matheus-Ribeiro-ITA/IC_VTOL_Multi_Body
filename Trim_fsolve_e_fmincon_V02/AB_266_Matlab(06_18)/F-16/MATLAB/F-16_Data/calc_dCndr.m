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

