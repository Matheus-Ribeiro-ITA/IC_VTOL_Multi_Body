function [Xdot,Y] = RotKinEuler(t,X)
    
    phi_rad=X(1);
    theta_rad=X(2);
    psi_rad=X(3);
    
    C_phi= DCM(1,phi_rad);
    C_theta=DCM(2,theta_rad);
    C_psi=DCM(3,psi_rad);
    
    C_bv = C_phi*C_theta*C_psi;
    
    e31=[1;0;0];
    e32=[0;1;0];
    e33=[0;0;1];
    
    KPhi =[e31 C_phi*e32 C_bv*e33];
    
    Omega_b=pqr_input(t);
    Phi_dot= KPhi\Omega_b; %% Resolve sistema linear, mais confiavel numericamente
    
    Xdot=Phi_dot;
    
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
    
    
    