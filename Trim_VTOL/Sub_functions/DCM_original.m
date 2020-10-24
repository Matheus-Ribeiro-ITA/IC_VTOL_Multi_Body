function C= DCM_original(n, mu_rad)

if length(n)==1
    switch n; %
        case 1
            C=[1 0 0 
                0 cos(mu_rad) sin(mu_rad)
                0 -sin(mu_rad) cos(mu_rad)];
        case 2
            C=[cos(mu_rad) 0 -sin(mu_rad)
                0 1 0
                sin(mu_rad) 0 cos(mu_rad)];
        case 3
            C=[ cos(mu_rad) sin(mu_rad) 0
                -sin(mu_rad) cos(mu_rad) 0
                0 0 1 ];
    end
else
    
    n=n/norm(n);
    C= (1-cos(mu_rad))*(n*n.')+cos(mu_rad)*eye(3)-sin(mu_rad)*skew(n);
end
