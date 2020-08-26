function J = PI_LQR_Integration(K,A,B,C,Q,R)

Ac = A -B*K*C;

In =eye(size(Ac));
J=0;
sys =ss(Ac,B,C,zeros(size( C(1,:),size(B,2) )));
for i=1:size(Ac,1)
    x0= In(:,1);
    [~,t,x] = initial(sys,x0,0:0.010:1000);
    x = x.';
    u=-K*C*x;
    vec_xTQx=zeros(length(t),1);
    vec_uTRu=zeros(length(t),1);
    for i_t=1:length(t)
        vec_xTQx(i_t)=x(:,i_t).'*Q*x(:,i_t);
        vec_uTRu(i_t)=u(:,i_t).'*R*u(:,i_t);
    end
    
    J= J + 1/2*trapz(t,vec_xTQx+vec_uTRu);
end
       