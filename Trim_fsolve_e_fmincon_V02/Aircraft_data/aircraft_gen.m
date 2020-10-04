function aircraft = aircraft_gen(m_carga,r_cg_0)

%% --------------------------------------------------------------------------------
% Geometri
% ---------------------------------------------------------------------

geom.b = 1.60;
geom.c = 0.25;
geom.S =  0.7;

%% --------------------------------------------------------------------------------
% Project parameters
% ---------------------------------------------------------------------


n_wing=4;
n_rotor=4;


%% --------------------------------------------------------------------------------
% INERTIA PARAMETERS
% ---------------------------------------------------------------------


% Massas

m_B=2.575 + m_carga;

m_w{1}=0.001;
m_w{2}=m_w{1};

m_w{3}=0.001;
m_w{4}=m_w{3};

for i=1:n_rotor
    m_r{i}=0.315;
end

% Centros de massa



r_w_0{1}= [0.020 0.240 -0.105]'; % Wing na vdd é a mesma coisa do rotor para nosso modelo
r_w_0{2}= [0.020 -0.240 -0.105]';
r_w_0{3}=[0.690 0.240 0.240]';
r_w_0{4}=[0.690 -0.240 0.240]';

r_r_0{1}=[0.0182 0.240 -0.101]';
r_r_0{2}=[0.0182 -0.240 -0.101]';
r_r_0{3}=[0.690 0.240 0.233]';
r_r_0{4}=[0.690 -0.240 0.233]';

m_total=m_B;
for i=1:n_wing
m_total=m_total+m_w{i};
end
for i=1:n_rotor
m_total=m_total+m_r{i};
end

r_b_0= r_cg_0*(m_total);

for i=1:n_rotor
r_b_0 = r_b_0 -r_r_0{i}*m_r{i};
end

r_b_0 = r_b_0/m_B;

% Aircraft center of mass

r_0=r_b_0*m_B;


for i=1:n_wing
r_0=r_0+m_w{i}*r_w_0{i};
end

for i=1:n_rotor
r_0=r_0+m_r{i}*r_r_0{i};
end

r_0=r_0/m_total;

% Body reference


% r_b_0=r_b_0-r_b_0;


for i=1:n_wing
r_w{i}=r_w_0{i}-r_b_0;
end

for i=1:n_rotor
r_r{i}=r_r_0{i}-r_b_0;
end



% Posicao da asa e rotor relativo aos pivots
r_pivot_0{1}= [-0.020 0.240 -0.03]';
r_w_pivot{1}=(r_w{1}+r_b_0) -r_pivot_0{1};

r_pivot_0{2}=[-0.020 -0.240 -0.029]';
r_w_pivot{2}=r_w{2}+r_b_0 -r_pivot_0{2};

r_pivot_0{3}=[0.690 0.240 0.310]';
r_w_pivot{3}=r_w{3}+r_b_0 -r_pivot_0{3};

r_pivot_0{4}= [0.690 -0.240 0.310]';
r_w_pivot{4}=r_w{4}+r_b_0 -r_pivot_0{4};


for i=1:4
    r_r_pivot{i}=r_r{i}+r_b_0 -r_pivot_0{i};
    r_pivot{i} = r_pivot_0{i} - r_b_0; 
end

% for i=5:6
%     r_r_pivot{i}=r_r{i}+r_b_0 -r_pivot{2};
% end


r_b=r_b_0-r_b_0;
%Inertia moments

% I_B=[0.368 -0.0004 0.118
%     -0.0004 0.332 8.2e-4
%      0.118 8.2e-4 0.554];

% Correção dos eixos do Catia
I_B=[0.368 0.0004 -0.118
    0.0004 0.332 -8.2e-4
     -0.118 -8.2e-4 0.554];

I_w{1}=[1e-10 0 0
        0 1e-10 0
         0 0 1e-10];
 
I_w{2}=[1e-10 0 0
        0 1e-10 0
         0 0 1e-10];
 
I_w{3}=[1e-10 0 0
        0 1e-10 0
         0 0 1e-10];

I_w{4}=[1e-10 0 0
        0 1e-10 0
         0 0 1e-10];



for i=1:n_rotor
I_r{i}=[2.55e-4   0   0
        0    2.55e-4  0
        0     0    2.55e-4];
end

I_total=I_B;


for i=1:n_rotor
R_T_r{i}=[r_r{i}(2)^2+r_r{i}(3)^2   -r_r{i}(1)*r_r{i}(2)   -r_r{i}(1)*r_r{i}(3)
        -r_r{i}(1)*r_r{i}(2)   r_r{i}(1)^2+r_r{i}(3)^2   -r_r{i}(2)*r_r{i}(3)         
         -r_r{i}(1)*r_r{i}(3)   -r_r{i}(2)*r_r{i}(3)    r_r{i}(1)^2+r_r{i}(2)^2];
end


for i=1:n_rotor
    I_total=I_total+(I_r{i} + m_r{i}*R_T_r{i});
end


aircraft=ws2struct();
end