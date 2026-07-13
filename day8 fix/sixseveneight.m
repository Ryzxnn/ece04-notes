%% TODO
% Implement gradient desc on 2d non-period grid with new f
% implemetn f on periodic torus
%     - f needs to satisfy wrap around constraint
%     - define dist on torus (check if wraparound is needed)
%%

clc
close all;
clear all;

x = linspace(0, 100);
y = linspace(0, 100);

[X,Y] = meshgrid(x, y);

low_x_bound = 40;
high_x_bound = 60;
low_y_bound = 20;
high_y_bound = 40;

unsafe = false(100,100);
unsafe(X < high_x_bound & X > low_x_bound & Y > low_y_bound & Y < high_y_bound) = true;

eps = 0.2;
sig = 4;

f = genPeriodicEikFun(100, unsafe, eps, sig);

figure;
imagesc(x, y, unsafe);

U = fastSweepingSolver(100,50,50,unsafe,40,1e-8, f)

figure; 
surf(X, Y, U);

figure; 
contourf(x, y, U, 50)
