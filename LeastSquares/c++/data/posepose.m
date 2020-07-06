clc
clear all
close all

syms z0 z1 z2 xi0 xi1 xi2 xj0 xj1 xj2
assume([z0 z1 z2 xi0 xi1 xi2 xj0 xj1 xj2],"real")

Zij = v2t([z0,z1,z2]);

Xi = v2t([xi0,xi1,xi2]);

Xj = v2t([xj0,xj1,xj2]);

eij = t2v(inv(Zij)*(inv(Xi)*Xj));
A = jacobian(eij,[xi0,xi1,xi2]);
B = jacobian(eij,[xj0,xj1,xj2]);
