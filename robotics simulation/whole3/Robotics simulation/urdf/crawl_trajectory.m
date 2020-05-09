% Initialize 
clear all;
clc;

i = linspace(300, 350, 51)

q1 = - 45 / 180 * pi / 50 * (i-300);
% q1 = -1/2 * sin((i-400)*0.05) - 1.2;
plot(i, q1);