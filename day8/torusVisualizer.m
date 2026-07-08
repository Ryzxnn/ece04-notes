function fig = torusVisualizer(theta1, theta2, theta1_goal, theta2_goal, U, contourLevel)
%TORUSVISUALIZER undefined
%   undefined
arguments (Input)
    theta1
    theta2
    theta1_goal
    theta2_goal
    U
    contourLevel=25
end

arguments (Output)
    fig
end

h = theta1(2) - theta2(1);
theta1(end+1) = theta1(end) + h;
theta2(end+1) = theta2(end) + h;

[Theta1,Theta2] = meshgrid(theta1, theta2);

U(:, end) = U(:, 1);
U(end, :) = U(1, :);
U(:, end+1) = U(:, 1);
U(end+1, :) = U(1, :);

fig = figure;
r = 2;
R = 5;

[C, ~] = contour(Theta1, Theta2, U, contourLevel);

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

end