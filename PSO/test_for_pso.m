clear all
close all
clc

[X,Y] = meshgrid(-20:0.5:40,-40:0.5:20);
Z = 10*X.^2 - 7*Y.^2 + 1000*sin(2*pi*0.1*Y);
surf(X,Y,Z)
xlabel('X')
ylabel('Y')
zlabel('Z')
