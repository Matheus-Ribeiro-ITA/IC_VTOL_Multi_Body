function R=DCM(euler)


% Direct Cossine Matrix

R=[cos(euler(3))*cos(euler(2))                                               sin(euler(3))*sin(euler(2))                                             -sin(euler(2))
   cos(euler(3))*sin(euler(2))*sin(euler(1))-sin(euler(3))*cos(euler(1))     sin(euler(3))*sin(euler(2))*sin(euler(1))-cos(euler(3))*cos(euler(1))   cos(euler(2))*sin(euler(2))
   cos(euler(3))*sin(euler(2))*cos(euler(1))+sin(euler(3))*sin(euler(1))     sin(euler(3))*sin(euler(2))*cos(euler(1))-cos(euler(3))*sin(euler(1))   cos(euler(2))*cos(euler(2))] ;

