function [u,udot]=cubic_transition(t,ti,tf,ui,uli,uf,ulf)

M = [1 ti ti^2 ti^3
    0 1 2*ti 3*ti^2
    1 tf tf^2 tf^3
    0 1 2*tf 3*tf^2];

b = [ui uli uf ulf].';

a = M\b;

u = a.'*[1 t t^2 t^3].';
udot = a.'*[0 1 2*t 3*t^2].';