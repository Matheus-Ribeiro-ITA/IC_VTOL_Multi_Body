function  [f,X,U,Y] = trim_fun_uni_body(x,aircraft,trim_par) 

% x(1,1) = de
% x(2,1) = Sigma
% x(3,1) = V_x
% x(4,1) = theta
% x(5,1) = throttle




X= [ 0;0;trim_par.h; x(3,1);0;sin(trim_par.alpha_eq)*trim_par.V; 0 ;x(4,1);0; 0;0;0; x(2,1); 0];
U = [x(5,1)
    x(1,1)
    0
    0];


[X_dot, Y]= dynamics_uni_body(0,X,U,aircraft);

    
C_chi = DCM_original(3,trim_par.chi);
C_gamma = DCM_original(2,trim_par.gamma);
dREOdt_v= C_chi.'*C_gamma.'*[trim_par.V;0;0];
x_dot_eq = dREOdt_v(1);
y_dot_eq = dREOdt_v(2);
h_dot_eq = dREOdt_v(3);


coord_flight = Y(9,1) - trim_par.beta;


f = [X_dot(1)- x_dot_eq
    X_dot(3)- h_dot_eq
    X_dot(4)
    X_dot(6)
    X_dot(11)];


    



    