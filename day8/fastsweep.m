clear all
close all
clc

N = 100; % number of grid points in each direction
theta1 = linspace(0,2*pi,N+1); % x-coordinates
theta1 = theta1(1:N);

theta2 = linspace(0,2*pi,N+1); % y-coordinates
theta2 = theta2(1:N);

h = theta1(2) - theta2(1); % grid spacing

[Theta1,Theta2] = meshgrid(theta1, theta2);


% Step 2: Choose a target point using array indices
theta1_goal = round(0);
theta2_goal = round();


[~, target_j] = min(abs(theta1-theta1_goal));
[~, target_i] = min(abs(theta2-theta2_goal));

% Physical coordinates of the target
unsafe = false(N, N);

unsafe(Theta2 > pi/2 & Theta2 < pi & Theta1 > pi/2 & Theta1 < pi) = true;

if ~isequal(size(unsafe), [N, N])
    error('unsafe must be an N-by-N matrix.');
end

if unsafe(target_i, target_j) == true
    error('The target configuration is inside the unsafe set.');
end

% Plot unsafe to inspect it.
figure;
imagesc(theta1,theta2,unsafe);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
xlabel('\theta_1');
ylabel('\theta_2');
title('Unsafe Set');

% Step 4: Initialize U
U = 3000 * ones(N, N);

U(target_i,target_j) = 0;
U(unsafe) = Inf;

if isinf(U(target_i,target_j))
    error('Target was marked as unsafe.');
end

figure;
imagesc(theta1,theta2,U);
set(gca,'YDir','normal');
axis equal tight;
colorbar;
xlabel('\theta_1');
ylabel('\theta_2');
title('aaa');

maxIter = 50;
tol = 1e-8;
for iter = 1:maxIter
    U_old = U;
    % Run four periodic sweeps.
    dirs_x = {1:N; N:-1:1; 1:N; N:-1:1};
    dirs_y = {1:N; 1:N; N:-1:1; N:-1:1};
    for d = 1:height(dirs_x)
        for i = dirs_y{d}
            for j = dirs_x{d}
                if i == target_i && j == target_j
                    continue;
                end
                if unsafe(i,j)
                    continue;
                end
                % Get periodic neighbors.
                [i_up,i_down,j_left,j_right] = getPeriodicNeighbors(i, j, N-1);
                % Read neighbor values.
                u_up = U(i_up, j);
                u_down = U(i_down, j);
                u_left = U(i, j_left);
                u_right = U(i, j_right);
                % Compute candidate update.
                u_candidate = localEikonalUpdate(min(u_right, u_left), min(u_up, u_down), h);
                % Accept the update only if it improves U.
                U(i,j) = min(U(i,j), u_candidate);
            end
        end
    end

    % Compute change only where both old and new values are finite.
    finite_mask = isfinite(U) & isfinite(U_old);
    change = max(abs(U(finite_mask) - U_old(finite_mask)));
    disp(['iter = ', num2str(iter), ', change = ', num2str(change)]);
    if change <= tol
        break;
    end
end

U

figure; 
surf(Theta1, Theta2, U)
colormap(jet);


figure; 
imagesc(theta1, theta2, U)

hold on;
contour(theta1, theta2, U, 15, 'LineWidth', 1.5, 'Color', 'k');
hold off;

figure;
r = 2;
R = 5;

[C, h] = contour(Theta1, Theta2, U, 30);

x = (R + r*cos(Theta1)).*cos(Theta2);
y = (R + r*cos(Theta1)).*sin(Theta2);
z = r*sin(Theta1);
surf(x, y, z, U, 'EdgeColor', 'none');

axis equal tight;
colormap(jet);
shading interp;
colorbar;

hold on;
startIndex = 1;
while startIndex < size(C, 2)
    numVertices = C(2, startIndex);

    t1_line = C(1, startIndex+1 : startIndex+numVertices);
    t2_line = C(2, startIndex+1 : startIndex+numVertices);

    X_line = (R + r * cos(t1_line)) .* cos(t2_line);
    Y_line = (R + r * cos(t1_line)) .* sin(t2_line);
    Z_line = r * sin(t1_line);

    plot3(X_line, Y_line, Z_line, 'LineWidth', 1, 'Color', 'k');

    startIndex = startIndex + numVertices + 1;
end

plot3((R + r * cos(theta1_goal)) .* cos(theta2_goal), (R + r * cos(theta1_goal)) .* sin(theta2_goal), r * sin(theta1_goal), 'x', 'Color', "white", 'MarkerSize', 10, 'LineWidth', 2)

hold off;



U = periodicFastSweepingSolver(theta1, theta2, target_i, target_j, unsafe, tol, maxIter);
torusVisualizer(theta1, theta2, theta1_goal, theta2_goal, U, 25)

