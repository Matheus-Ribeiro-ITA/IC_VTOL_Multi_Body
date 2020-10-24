function  [f,X,U] = trim_function(x,aircraft,trim_par) 

% x=[u w q theta v phi p r psi power dt de da dr].';

X= [ x(1:4); trim_par.h; 0; x(5:9); 0; x(10)];
U= x(11:14);

if not(isfield(trim_par,'Vw_v'))
    [Xdot, Y]= dynamics(0,X,U,aircraft);
else
    [Xdot, Y]= dynamics(0,X,U,aircraft,trim_par.Vw_v);
end
C_chi = DCM(3,trim_par.chi_deg*pi/180);
C_gamma = DCM(2,trim_par.gamma_deg*pi/180);
dREOdt_v= C_chi.'*C_gamma.'*[trim_par.V;0;0];
x_dot_eq = dREOdt_v(1);
y_dot_eq = dREOdt_v(2);
h_dot_eq = dREOdt_v(3);

if strcmpi(trim_par.coord_flag,'beta')
    coord_flight = Y(3) - trim_par.beta_deg;
elseif strcmpi(trim_par.coord_flag,'nyp')
    coord_flight = Y(end-1) - trim_par.nyp;
end

f = [ Xdot(1:3)
    Xdot(4)- trim_par.theta_dot_deg_s
    Xdot(5)- h_dot_eq
    Xdot(6)- x_dot_eq
    Xdot(7)
    Xdot(8)- trim_par.phi_dot_deg_s
    Xdot(9:10)
    Xdot(11)- trim_par.psi_dot_deg_s
    Xdot(12)- y_dot_eq
    Xdot(13)
    coord_flight];

% Y(3)-3 % beta_deg_eq = 3 , por exemplo (equilibrio em voo derrapado)
    
    



    