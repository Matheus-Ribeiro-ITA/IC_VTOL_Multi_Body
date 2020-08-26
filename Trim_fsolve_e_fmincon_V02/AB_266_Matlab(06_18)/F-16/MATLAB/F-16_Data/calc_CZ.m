function CZ_calc = calc_CZ(alpha,beta,el) % CZ0

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