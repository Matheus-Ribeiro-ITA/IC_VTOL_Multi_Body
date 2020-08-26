clear
close all
clc

load ABCD.mat A B C D
n=length(A);


sel_long = [6 11 14]; % 2 w, 3 q , 14 profudor
sel_ctrl = 2; % u_e e ??

for i=1:n
Along{i} =A{i}(sel_long,sel_long);
Blong{i} = B{i}(sel_long,sel_ctrl);
Clong{i} = C{i}(:,sel_long);
Dlong{i} = D{i}(:,sel_ctrl);
end




figure

for i=1:n
plot(real(eig(A{i})),imag(eig(A{i})),'r.','MarkerSize',5)
hold on
end
grid on
legend('Open loop')