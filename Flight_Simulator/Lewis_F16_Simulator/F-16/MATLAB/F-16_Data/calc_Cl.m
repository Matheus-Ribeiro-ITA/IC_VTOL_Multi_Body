function Cl_calc = calc_Cl(alpha,beta) % CL0

% 3. ed.:
% Corre��o implementada por Juliano Paulino e Guilherme Barbosa:
A = [zeros(1,12)
    -.001 -.004 -.008 -.012 -.016 -.019 -.020 -.020 -.015 -.008 -.013 -.015
    -.003 -.009 -.017 -.024 -.030 -.034 -.040 -.037 -.016 -.002 -.010 -.019
    -.001 -.010 -.020 -.030 -.039 -.044 -.050 -.049 -.023 -.006 -.014 -.027
    .000 -.010 -.022 -.034 -.047 -.046 -.059 -.061 -.033 -.036 -.035 -.035
    .007 -.010 -.023 -.034 -.049 -.046 -.068 -.071 -.060 -.058 -.062 -.059
    .009 -.011 -.023 -.037 -.050 -.047 -.074 -.079 -.091 -.076 -.077 -.076].';
% 2. ed.:
% Corre��o implementada por Juliano Paulino e Guilherme Barbosa:
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