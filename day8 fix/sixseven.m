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

marg = 5;

low_x_bound = 40;
high_x_bound = 60;
low_y_bound = 20;
high_y_bound = 40;

unsafe = false(100,100);
unsafe(X < high_x_bound & X > low_x_bound & Y > low_y_bound & Y < high_y_bound) = true;

% f = [ones(100, 50), 0.5*ones(100, 50)];

f = ones(100, 100);

eps = 0.2;
sig = 4;

function d = distaaa(x, y, low_x_bound, high_x_bound, low_y_bound, high_y_bound)
    ds = [];
    for i = low_y_bound-5:high_y_bound+5
        for j = low_x_bound-5:high_x_bound+5
            ds(end+1) = norm([x-j, y-i]);
        end
    end
    d = min(ds);
end

for i = 1:100
    for j = 1:100
        f(i, j) = ( ...
            1+(1+eps*(i+j)) ...
            * exp(-distaaa(j, i, low_x_bound, high_x_bound, low_y_bound, high_y_bound)/sig) ...
        );
    end
end

figure;
imagesc(x, y, unsafe);

U = fastSweepingSolver(100,50,50,unsafe,40,1e-8, f)

% figure; 
% surf(U)
% grid on;

figure; 
surf(X, Y, U);

figure; 
contourf(x, y, U, 50)
