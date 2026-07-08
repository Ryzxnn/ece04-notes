clear all
close all
clc

% Step 1: Create a square grid on [0,1] x [0,1]
N = 101; % number of grid points in each direction
x = linspace(0,1,N); % x-coordinates
y = linspace(0,1,N); % y-coordinates
h = x(2) - x(1) % grid spacing
% Fill this in:
% Hint: use meshgrid.
[X,Y] = meshgrid(x, y)

% Step 2: Choose a target point using array indices
target_i = round(N/2); % row index of target
target_j = round(N/5); % column index of target
% Physical coordinates of the target
xg = x(target_j);
yg = y(target_i);

% Step 3: Initialize U
U = Inf(N,N); % unknown values are initialized to infinity
% Fill this in:
% Set the value at the target to zero.
U(target_i, target_j) = 0;

% Step 4: Exact Euclidean distance to the target
% Fill this in:
% Hint: use sqrt and .^2.

U_exact = sqrt((X-xg).^2 + (Y-yg).^2)


figure;
imagesc(x,y,U_exact);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
title('Exact Distance to the Target');
xlabel('x');
ylabel('y');
figure;
contour(X,Y,U_exact);

axis equal tight;
colorbar;
title('Contours of Exact Distance');
xlabel('x');
ylabel('y');

% Step 5: Simple neighbor update at one grid point
i = target_i + 1;
j = target_j;
% Fill this in:
% Find the smallest of the four neighboring values.
best_neighbor = min([U(i-1, j), U(i+1, j), U(i, j-1), U(i, j+1)]);
    % Candidate value from the simple update.
u_candidate = best_neighbor+h;
    % Only accept the update if it improves the old value.
U(i,j) = min(U(i,j), u_candidate);

% Step 6: One sweep using the simple update
U_simple = Inf(N,N);
U_simple(target_i,target_j) = 0;
for i = 2:N-1
    for j = 2:N-1
        % Do not update the target point.
        if i == target_i && j == target_j
            continue;
        end
        % Find the best of the four neighbors.
        best_neighbor = min([U_simple(i-1, j), U_simple(i+1, j), U_simple(i, j-1), U_simple(i, j+1)]);
        % Use simple update.
        u_candidate = best_neighbor+h;
            % Accept only if candidate is better.
        U_simple(i,j) = min(U_simple(i,j), u_candidate);
    end
end

figure;
imagesc(x,y,U_simple);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
title('After One Simple Sweep');
xlabel('x');
ylabel('y');

U_simple = Inf(N,N);
U_simple(target_i,target_j) = 0;

function U_simple = sweep(U_simple, range_i, range_j, target_i, target_j, h)
    for i = range_i
        for j = range_j
            % Do not update the target point.
            if i == target_i && j == target_j
                continue;
            end
            % Find the best of the four neighbors.
            best_neighbor = min([U_simple(i-1, j), U_simple(i+1, j), U_simple(i, j-1), U_simple(i, j+1)]);
            % Use simple update.
            u_candidate = best_neighbor+h;
            % Accept only if candidate is better.
            U_simple(i,j) = min(U_simple(i,j), u_candidate);
        end
    end
end


U_simple = sweep(U_simple, 2:N-1, 2:N-1, target_i, target_j, h);
U_simple = sweep(U_simple, N-1:-1:2, 2:N-1, target_i, target_j, h);
U_simple = sweep(U_simple, 2:N-1, N-1:-1:2, target_i, target_j, h);
U_simple = sweep(U_simple,  N-1:-1:2,  N-1:-1:2, target_i, target_j, h);

figure;
imagesc(x,y,U_simple);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
title('Simple Update After 4 Sweeps');
xlabel('x');
ylabel('y');

% Step 10: Fast Sweeping update at one grid point
i = target_i + 1;
j = target_j;
% Fill these in:
% a should be the best horizontal neighbor.
% b should be the best vertical neighbor.
a = min(U(i, j+1), U(i, j-1));
b = min(U(i+1, j), U(i-1, j));
u_candidate = localEikonalUpdate(a,b,h);
U(i,j) = min(U(i,j), u_candidate);

U = Inf(N,N);
U(target_i,target_j) = 0;

function U = sweepEik(U, range_i, range_j, target_i, target_j, h)
    for i = range_i
        for j = range_j
            if i == target_i && j == target_j
                continue;
            end
            a = min(U(i, j+1), U(i, j-1));
            b = min(U(i+1, j), U(i-1, j));
            u_candidate = localEikonalUpdate(a,b,h);
            U(i,j) = min(U(i,j), u_candidate);
        end
    end
end

U = sweepEik(U, 2:N-1, 2:N-1, target_i, target_j, h);
U = sweepEik(U, N-1:-1:2, 2:N-1, target_i, target_j, h);
U = sweepEik(U, 2:N-1, N-1:-1:2, target_i, target_j, h);
U = sweepEik(U,  N-1:-1:2,  N-1:-1:2, target_i, target_j, h);

% Step 12: Repeat four sweeps until convergence
U = Inf(N,N);
U(target_i,target_j) = 0;
maxIter = 50;
tol = 1e-8;
for iter = 1:maxIter
    U_old = U;
    U = sweepEik(U, 2:N-1, 2:N-1, target_i, target_j, h);
    U = sweepEik(U, N-1:-1:2, 2:N-1, target_i, target_j, h);
    U = sweepEik(U, 2:N-1, N-1:-1:2, target_i, target_j, h);
    U = sweepEik(U,  N-1:-1:2,  N-1:-1:2, target_i, target_j, h);
    % Fill this in:
    % Compute the largest change in U during this iteration.
    change = max(abs(U-U_old));

    if change < tol
        disp(['Converged after ', num2str(iter), ' iterations.'])
        break;
    end
end
U

figure;
imagesc(x,y,U);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
title('Fast Sweeping Solution');
xlabel('x');
ylabel('y');

figure;
contour(X,Y,U,30);
axis equal tight;
colorbar;
title('Contours of Fast Sweeping Solution');
xlabel('x');
ylabel('y');

% Step 14: Compare numerical solution to exact solution
% Fill this in:
error = abs(U-U_exact);

figure;
imagesc(x,y,error);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
title('Error Compared with Exact Distance');
xlabel('x');
ylabel('y');

figure;
axis equal tight;
surf(X, Y, U)

hold on;
surf(X, Y, U_exact)
hold off;

% Step 15: Create a rectangular obstacle
obstacle = false(N,N);
% This creates a vertical rectangular obstacle.
obstacle(Y > 0.35 & Y < 0.65 & X > 0.45 & X < 0.55) = true;
% Make sure the target is not inside the obstacle.
obstacle(target_i,target_j) = false;

function U = sweepEik2(U, obstacle, range_i, range_j, target_i, target_j, h)
    for i = range_i
        for j = range_j
            if i == target_i && j == target_j
                continue;
            end
            if obstacle(i,j)
                continue;
            end
            a = min(U(i, j+1), U(i, j-1));
            b = min(U(i+1, j), U(i-1, j));
            u_candidate = localEikonalUpdate(a,b,h);
            U(i,j) = min(U(i,j), u_candidate);
        end
    end
end

U = Inf(N,N);
U(target_i,target_j) = 0;
maxIter = 50;
tol = 1e-8;
for iter = 1:maxIter
    U_old = U;
    U = sweepEik2(U, obstacle, 2:N-1, 2:N-1, target_i, target_j, h);
    U = sweepEik2(U, obstacle, N-1:-1:2, 2:N-1, target_i, target_j, h);
    U = sweepEik2(U, obstacle, 2:N-1, N-1:-1:2, target_i, target_j, h);
    U = sweepEik2(U, obstacle, N-1:-1:2,  N-1:-1:2, target_i, target_j, h);
    % Fill this in:
    % Compute the largest change in U during this iteration.
    change = max(abs(U-U_old));

    if change < tol
        disp(['Converged after ', num2str(iter), ' iterations.'])
        break;
    end
end


% Step 16: Plot the Solution with the Obstacle
function fig = plotSweep(U, X, Y, obstacle, tit)
fig = figure;
U_plot = U;
U_plot(obstacle) = NaN;
surf(X,Y,U_plot);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
title(tit);
xlabel('x');
ylabel('y');
end

p = plotSweep(U, X, Y, obstacle, "Fast Sweeping Solution with Obstacle")

function fig = contourSweep(U, X, Y, obstacle, tit)
fig = figure;
U_plot = U;
U_plot(obstacle) = NaN;
contour(X,Y,U_plot,30);
axis equal tight;
colorbar;
title(tit);
xlabel('x');
ylabel('y');
end

p = contourSweep(U, X, Y, obstacle, "Contours with Obstacle")


% Step 17
U = Inf(N,N);
U(target_i,target_j) = 0;
U = fastSweepIteration(U, h, target_i, target_j, obstacle)
p = plotSweep(U, X, Y, obstacle, "")
p = contourSweep(U, X, Y, obstacle, "")

% Step 18
U = fastSweepingSolver(N, target_i, target_j, obstacle, maxIter, tol);
p = plotSweep(U, X, Y, obstacle, "")
p = contourSweep(U, X, Y, obstacle, "")
