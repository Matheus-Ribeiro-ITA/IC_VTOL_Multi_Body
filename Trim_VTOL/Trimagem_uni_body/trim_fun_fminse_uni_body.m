function [points,X_eq,U_eq,Y_eq] = trim_fun_fminse_uni_body(alpha_eq,V_eq,aircraft,trim_par,ini_0)


trim_par.alpha_eq=alpha_eq;


x_eq_0 =zeros(5,1);    
x_eq_0(1) = ini_0.U_0(2,1);
x_eq_0(2) = ini_0.X_0(13,1);
x_eq_0(3) = V_eq;
x_eq_0(4) = ini_0.X_0(8,1); %Theta
x_eq_0(5) = ini_0.U_0(1,1);



options = optimset('Display','none','TolFun',1e-10,'TolX',1e-10);
%options = optimset('Display','iter','TolFun',1e-10,'TolX',1e-10);
% options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
[x_eq,fval,exitflag,output,jacobian] = fsolve(@trim_fun_uni_body,x_eq_0,options,aircraft,trim_par);


[~,X_eq,U_eq,Y_eq]  = trim_fun_uni_body(x_eq,aircraft,trim_par);


% Trigonometry correction

while X_eq(13,1)*180/pi>=360
    X_eq(13,1)=X_eq(13,1)-2*pi;
end

while X_eq(13,1)*180/pi<0
    X_eq(13,1)=X_eq(13,1)+2*pi;
end


if X_eq(13,1)*180/pi<270 && X_eq(13,1)*180/pi>180 && U_eq(1)<0
    U_eq(1)=-U_eq(1);
    X_eq(13,1)=X_eq(13,1)-pi;
end

points=0;

%Penalization

if X_eq(13,1)>pi/2
    points = abs(X_eq(13,1)-2*pi)*100*1;
else
    points = 0;
end

if U_eq(1,1)<0
    points = points + abs(U_eq(1,1))*100*1;
end



% Pontuation

points = points + abs(U_eq(1,1))*100;