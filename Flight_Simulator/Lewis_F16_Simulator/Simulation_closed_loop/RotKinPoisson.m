function [Xdot,Y] = RotKinPoisson(t,X)

C11 =X(1);
C21 =X(2);
C31 =X(3);
C12 =X(4);
C22 =X(5);
C32 =X(6);
C13 =X(7);
C23 =X(8);
C33 =X(9);

C_bv= [C11 C12 C13
        C21 C22 C23
        C31 C32 C33];
    
Omega_b=pqr_input(t);
C_bv_dot= -skew(Omega_b)*C_bv;

Xdot=[ C_bv_dot(:,1)
    C_bv_dot(:,2)
    C_bv_dot(:,3)];

phi_rad=atan2(C(2,3),C(3,3));
theta_rad=-asin(C(1,3));
psi_rad=atan2(C(1,2),C(1,1));

 phi_deg=phi_rad*180/pi;
 theta_deg=theta_rad*180/pi;
 psi_deg=psi_rad*180/pi;
    
Y=[C_bv(:,1)
        C_bv(:,2)
        C_bv(:,3)
        Omega_b*180/pi;
        phi_deg
        theta_deg
        psi_deg];