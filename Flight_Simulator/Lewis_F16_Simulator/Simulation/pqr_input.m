function Omega = pqr_input(t)

ti = 1;
tf = 3;

pf_deg_s = -5;
qf_deg_s = 5;
rf_deg_s = 5;

if t<=ti
    Omega = zeros(3,1);
elseif t>ti && t<=tf
    p_rad_s = cubic_transition(t,ti,tf,0,0,pf_deg_s*pi/180,0);
    q_rad_s = cubic_transition(t,ti,tf,0,0,qf_deg_s*pi/180,0);
    r_rad_s = cubic_transition(t,ti,tf,0,0,rf_deg_s*pi/180,0);
    Omega = [p_rad_s; q_rad_s; r_rad_s];
else
    p_rad_s = pf_deg_s*pi/180;
    q_rad_s = qf_deg_s*pi/180;
    r_rad_s = rf_deg_s*pi/180;
    Omega = [p_rad_s; q_rad_s; r_rad_s];
end