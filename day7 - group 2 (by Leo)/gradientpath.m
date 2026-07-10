clear all
close all
clc

% Step 1: Create a square grid on [0,1] x [0,1]
N = 101; % number of grid points in each direction
x = linspace(0,1,N); % x-coordinates
y = linspace(0,1,N); % y-coordinates
h = x(2) - x(1); % grid spacing
% Fill this in:
% Hint: use meshgrid.
[X,Y] = meshgrid(x, y);

% Step 2: Choose the goal and the initial point
goal = [0.8, 0.8]; % [x_goal, y_goal]
x0 = [0.2, 0.2]; % [x_initial, y_initial]

% Step 3: Create a simple value function
xg = goal(1);
yg = goal(2);
% Fill in the formula:
% U = (X - xg)^2 + (Y - yg)^2
%
% Hint:
% X and Y are matrices, so use .^2 instead of ^2.
U = (X - xg).^2 + (Y - yg).^2;

% Step 4: Plot contours of U
figure;
contour(X,Y,U,30);
hold on;
plot(goal(1),goal(2),'rx','MarkerSize',12,'LineWidth',2);
plot(x0(1),x0(2),'bo','MarkerSize',8,'LineWidth',2);
axis equal tight;
colorbar;
xlabel('x');
ylabel('y');
title('Contours of U');
legend('Contours of U','Goal','Initial point');

% Step 5: Compute numerical gradients of U
%
% Hint:
% Use MATLAB's gradient function.
%
% Important:
% MATLAB arrays are indexed by rows and columns.
% Rows correspond to the y-direction.
% Columns correspond to the x-direction.
%
% Therefore, we store the outputs as [Uy,Ux].
[Ux, Uy] = gradient(U, h);

% Step 6: Plot negative gradient field
skip = 5; % plot every 5th arrow to keep the figure clean
figure;
contour(X,Y,U,30);
hold on;
% Fill in the quiver command:
% Hint:
% quiver(Xsub,Ysub,Usub,Vsub) plots arrows.
% We want arrows in the direction [-Ux, -Uy].
% Use 1:skip:end to avoid plotting too many arrows.
quiver(X, Y, -Ux, -Uy);
plot(goal(1),goal(2),'rx','MarkerSize',12,'LineWidth',2);
plot(x0(1),x0(2),'bo','MarkerSize',8,'LineWidth',2);
axis equal tight;
xlabel('x');
ylabel('y');
title('Negative Gradient Field');

% Step 7: Initialize simulation
dt = 0.01; % time step
Nsteps = 500; % maximum number of steps
path = zeros(Nsteps,2); % each row stores [x,y] at one time step
% Fill this in:
% Store the initial point in the first row of path.
path(1,:) = x0;

% Step 8: Interpolate gradient at one point
%
% Suppose current stores the current particle position.
current = path(1,:);
% Fill these in:
% Hint:
% Use interp2(X,Y,Ux,current(1),current(2),'linear',0)
% to estimate dU/dx at the current point.
%
% Use a similar line for dU/dy.
grad_x = interp2(X, Y, Ux, current(1), current(2), 'linear', 0);
grad_y = interp2(X, Y, Uy, current(1), current(2), 'linear', 0);
grad = [grad_x, grad_y];

% Step 9: One Euler update
%
% Mathematical update:
% next = current - dt * grad
next = current - dt * grad;

% Step 10: Main simulation loop
for k = 1:Nsteps-1
    % 1. Read current position from path
    current = path(k, :);
    % 2. Interpolate the gradient at current position
    grad_x = interp2(X, Y, Ux, current(1), current(2), 'linear', 0);
    grad_y = interp2(X, Y, Uy, current(1), current(2), 'linear', 0);
    grad = [grad_x, grad_y];
    % 3. Euler update in the negative-gradient direction
    next = current - dt * grad;
    % 4. Store next position
    path(k+1,:) = next;
    % 5. Stop if close to the goal
    % Hint: use norm(next - goal)
    if norm(next - goal) < h
        path = path(1:k+1,:);
        disp('Reached the goal!');
    break;
    end
    % 6. Stop if outside the square [0,1] x [0,1]
    if next(1) < 0 || next(1) > 1 || next(2) < 0 || next(2) > 1
        path = path(1:k+1,:);
        disp('Left the domain!');
        break;
    end
end

% Step 11: Plot trajectory on contours of U
figure;
contour(X,Y,U,30);
hold on;
plot(path(:,1),path(:,2),'k-','LineWidth',2);
plot(goal(1),goal(2),'rx','MarkerSize',12,'LineWidth',2);
plot(x0(1),x0(2),'bo','MarkerSize',8,'LineWidth',2);
axis equal tight;
colorbar;
xlabel('x');
ylabel('y');
title('Gradient Descent Trajectory');
legend('Contours of U','Trajectory','Goal','Initial point');

% Step 12: Add an artificial obstacle penalty
obstacle_center = [0.5, 0.5];
xo = obstacle_center(1);
yo = obstacle_center(2);
alpha = 0.001; % obstacle strength
epsilon = 1e-4; % prevents division by zero
% Fill these in:
% attractive_part should pull the particle to the goal.
% repulsive_part should become large near the obstacle.
attractive_part = (X - xg).^2 + (Y - yg).^2
repulsive_part = alpha ./ ((X - xo).^2 + (Y - yo).^2 + epsilon)                       
U = attractive_part + repulsive_part;

% Step 13: Define and draw a circular obstacle
obstacle_radius = 0.12                                                                                                                                                        ;
obstacle = ((X - xo).^2 + (Y - yo).^2) <= obstacle_radius^2;
theta = linspace(0,2*pi,200);

obs_x = xo + obstacle_radius*cos(theta);
obs_y = yo + obstacle_radius*sin(theta);

% Step 14: Recompute gradient and rerun simulation
% Fill this in:
% Recompute the gradient of the new U.
figure;
contour(X,Y,U,40);



[Uy,Ux] = gradient(U, h);

figure; 
quiver(X, Y, -Ux, -Uy);
axis equal tight;
xlabel('x');
ylabel('y');
title('Negative Gradient Field');

% Reset the path.
path = zeros(Nsteps,2);
path(1,:) = x0;
for k = 1:Nsteps-1
    % Same logic as before:
    % 1. current position
    % 2. interpolate gradient
    % 3. Euler update
    % 4. store next position
    % 5. stop near goal
    % 6. stop outside domain
    current = path(k, :);
    grad_x = interp2(X, Y, Ux, current(1), current(2), 'linear', 0);
    grad_y = interp2(X, Y, Uy, current(1), current(2), 'linear', 0);
    grad = [grad_x, grad_y];
    next = current - dt * grad;
    path(k+1,:) = next;
    if norm(next - goal) < h
        path = path(1:k+1,:);
        disp("Reached the goal!");
        break;
    end
    if next(1) < 0 || next(1) > 1 || next(2) < 0 || next(2) > 1
        path = path(1:k+1,:);
        disp("Left the domain!");
        break;
    end
end

% Step 15: Plot trajectory with obstacle
figure;
contour(X,Y,U,40);
hold on;
plot(path(:,1),path(:,2),'k-','LineWidth',2);
plot(goal(1),goal(2),'rx','MarkerSize',12,'LineWidth',2);
plot(x0(1),x0(2),'bo','MarkerSize',8,'LineWidth',2);
plot(obs_x,obs_y,'r-','LineWidth',2);
axis equal tight;
colorbar;
xlabel('x');
ylabel('y');
title('Trajectory with Artificial Obstacle Penalty');
legend('Contours of U','Trajectory','Goal','Initial point','Obstacle');

