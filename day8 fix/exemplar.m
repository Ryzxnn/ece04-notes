clear all
% close all
clc

N = 100; % number of grid points in each direction
theta1 = linspace(0,2*pi,N+1); % x-coordinates
theta1 = theta1(1:N);

theta2 = linspace(0,2*pi,N+1); % y-coordinates
theta2 = theta2(1:N);

theta1_goal = round(0);
theta2_goal = round(1);

[Theta1, Theta2] = meshgrid(theta1, theta2);

[~, target_j] = min(abs(theta1-theta1_goal));
[~, target_i] = min(abs(theta2-theta2_goal));

unsafe = false(N, N);
unsafe(Theta2 > pi/12 & Theta2 < pi/6 & Theta1 > -pi/6 & Theta1 < pi/6) = 1;

figure;
imagesc(theta1, theta2, unsafe)

tol = 1e-8;
maxIter = 50;

U = periodicFastSweepingSolver(theta1, theta2, target_i, target_j, unsafe, tol, maxIter);

torusVisualizer(theta1, theta2, theta1_goal, theta2_goal, U, 25)

[gy, gx] = gradient(U, theta1(2)-theta1(1));
figure; 
quiver(Theta1, Theta2, -gx, -gy);

figure; 
contourf(theta1, theta2, U, 50)
