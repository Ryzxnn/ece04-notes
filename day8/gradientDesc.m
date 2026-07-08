clear all
close all
clc

N = 100; % number of grid points in each direction
theta1 = linspace(0,2*pi,N+1); % x-coordinates
theta1 = theta1(1:N);

theta2 = linspace(0,2*pi,N+1); % y-coordinates
theta2 = theta2(1:N);

theta1_goal = round(pi/2);
theta2_goal = round(2);

[Theta1, Theta2] = meshgrid(theta1, theta2);

[~, target_j] = min(abs(theta1-theta1_goal));
[~, target_i] = min(abs(theta2-theta2_goal));

unsafe = false(N, N);
unsafe(Theta2 > 0.5 & Theta2 < 3 & Theta1 > 1 & Theta1 < 1.5) = true;

tol = 1e-8;
maxIter = 50;
U = periodicFastSweepingSolver(theta1, theta2, target_i, target_j, unsafe, tol, maxIter);
    
figure; 
imagesc(theta1, theta2, U);

% torusVisualizer(theta1, theta2, theta1_goal, theta2_goal, U, 25)

t0 = [5.5, 5.5];

goal = [theta1_goal, theta2_goal];

h = theta1(2)-theta1(1);

U(isinf(U)) = NaN;
[Ux, Uy] = gradient(U, h/2);

figure; 
quiver(Theta1, Theta2, -Ux, -Uy);

dt=0.0001;
Nsteps = 10/dt;
goal_tol = 0.018;

path = zeros(Nsteps,2);
path(1,:) = t0;
for k = 1:Nsteps-1
    current = path(k, :);
    grad_x = interp2(Theta1, Theta2, Ux, current(1), current(2), 'linear', 0);
    grad_y = interp2(Theta1, Theta2, Uy, current(1), current(2), 'linear', 0);
    grad = [grad_x, grad_y];
    next = current - dt * grad; 
    path(k+1,:) = mod(next, 2*pi-h*1.1);
    path = path(1:k+1,:);
    if norm(next - goal) <= goal_tol
        path = path(1:k+1,:);
        disp("Reached the goal!")
        break;
    end
end

figure;
contour(Theta1,Theta2,U,40);
hold on;
quiver(Theta1, Theta2, -Ux, -Uy);

breakIdx = find(sqrt(sum((diff(path, 1, 1)).^2, 2)) > 3);
paths = cell(length(breakIdx)+1, 1);
startRow = 1;
for k = 1:length(breakIdx)
    paths{k} = path(startRow:breakIdx(k), :);
    startRow = breakIdx(k) + 1;
end
paths{end} = path(startRow:end, :);

for p = 1:length(paths)
    plot(paths{p}(:,1),paths{p}(:,2),'k-','LineWidth',2);
end

plot(goal(1),goal(2),'rx','MarkerSize',12,'LineWidth',2);
plot(t0(1),t0(2),'bo','MarkerSize',8,'LineWidth',2);
axis equal tight;
colorbar;
xlabel('x');
ylabel('y');
title('Trajectory with Artificial Obstacle Penalty');

