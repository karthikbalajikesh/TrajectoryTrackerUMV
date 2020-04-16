clear 
close all
clc

deltat = 0.1;
t = 0: 0.1 : 3;

A = [
a = 0: 0.06: 3;

v = zeros(size(t));
s = zeros(size(t));

for i = 2:length(t)
    
    v(i) = v(i-1) +( a(i-1)*deltat);
    s(i) = s(i-1) + (v(i)* deltat);
end
