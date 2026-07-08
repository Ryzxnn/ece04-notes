clear all
close all
clc

function T = Tx(a)
    T = [ 1, 0, 0, a; 
          0, 1, 0, 0; 
          0, 0, 1, 0; 
          0, 0, 0, 1];
end
function T = Ty(b)
    T = [ 1, 0, 0, 0; 
          0, 1, 0, b; 
          0, 0, 1, 0; 
          0, 0, 0, 1];
end
function T = Tz(c)
    T = [ 1, 0, 0, 0; 
          0, 1, 0, 0; 
          0, 0, 1, c; 
          0, 0, 0, 1];
end
function T = Rx(theta)
    T = [ 1, 0, 0, 0;
          0, cos(theta), -sin(theta), 0;
          0, sin(theta), cos(theta), 0;
          0, 0, 0, 1 ];
end
function T = Ry(theta)
    T = [ cos(theta), 0, -sin(theta), 0;
          0, 1, 0, 0;
          sin(theta), 0, cos(theta), 0;
          0, 0, 0, 1 ];
end
function T = Rz(theta)
    T = [ cos(theta), -sin(theta), 0, 0;
          sin(theta), cos(theta), 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1 ];
end

e_x = [1; 0; 0];
e_y = [0; 1; 0];
e_z = [0; 0; 1];

Tx(1)

Rx(0)

Rz(pi/2)*[e_x; 1]

Ry(pi/2)*[e_x; 1]

Rx(pi/2)*[e_y; 1]

%%

L1 = 1;
theta1 = 2*pi-0.5;

L2 = 3;
theta2 = pi/4;

L3 = 2;
theta3 = -pi/2;

T_01 = Tz(L1) * Rz(theta1)
T_12 = Ry(theta2) * Tx(L2)
T_23 = Ry(theta3) * Tx(L3)

T_02 = T_01 * T_12
T_03 = T_02 * T_23

p0 = [0;0;0;1];

p1 = T_01*p0
p2 = T_02*p0
p3 = T_03*p0

x3 = [p0(1, :), p1(1, :), p2(1, :),p3(1, :)];
y3 = [p0(2, :), p1(2, :), p2(2, :),p3(2, :)];
z3 = [p0(3, :), p1(3, :), p2(3, :),p3(3, :)];

figure; 
scatter3(x3, y3, z3)

hold on;
plot3(x3, y3, z3)

xlim([-7, 7]);
ylim([-7, 7]);
zlim([-7, 7]);

hold off;


x = L2 * cos(theta2) + L3 * cos(theta2 + theta3)
y = L2 * sin(theta2) + L3 * sin(theta2 + theta3)

% f-kinematics consistent with results (mapping x y to the 3d x-z plane and
% accounting for origin frame offset

%%
L1 = 1;
theta1 = 0;

L2 = 3;
theta2 = pi/4;

L3 = 2;
theta3 = -pi/2;

f = figure; 

axis tight manual
ax = gca;
ax.NextPlot = 'replaceChildren';

loops = 80;
M(loops) = struct('cdata', [], 'colormap', []);

f.Visible = 'off';

for i = 1:loops
    theta1 = theta1 + 2*pi/loops;
    theta2 = cos(4*pi*i/loops)*pi/4;
    theta3 = sin(4*pi*i/loops)*pi/4;


    T_01 = Tz(L1) * Rz(theta1);
    T_12 = Ry(theta2) * Tx(L2);
    T_23 = Ry(theta3) * Tx(L3);
    
    T_02 = T_01 * T_12;
    T_03 = T_02 * T_23;
    
    p0 = [0;0;0;1];
    
    p1 = T_01*p0;
    p2 = T_02*p0;
    p3 = T_03*p0;
    
    x3 = [p0(1, :), p1(1, :), p2(1, :),p3(1, :)];
    y3 = [p0(2, :), p1(2, :), p2(2, :),p3(2, :)];
    z3 = [p0(3, :), p1(3, :), p2(3, :),p3(3, :)];
    
    scatter3(x3, y3, z3)

    hold on;
    plot3(x3, y3, z3)

    xlim([-7, 7]);
    ylim([-7, 7]);
    zlim([-7, 7]);
    hold off;

    drawnow

    M(i) = getframe;
end

f.Visible = 'on';
movie(M);

%%
figure;
movie(M);
