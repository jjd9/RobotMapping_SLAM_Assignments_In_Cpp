
clc
clear all
close all

syms z0 z1 x0 x1 x2 lm0 lm1
assume([z0 z1 x0 x1 x2 lm0 lm1],"real")

x = [x0,x1,x2]';
lm = [lm0,lm1]';
z = [z0,z1]';

R = v2t(x);
R = R(1:2,1:2);

eij = R'*(lm-x(1:2)) - z;

A = jacobian(eij,x);
B = jacobian(eij,lm);
