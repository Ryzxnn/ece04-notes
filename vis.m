function vis(phi_meas)
persistent phi_history fig path_line current_pos

phi_meas = phi_meas(:);

if isempty(fig) || ~isgraphics(fig)
    phi_history = zeros(numel(phi_meas), 0);
    fig = figure('Color', 'w', 'Name', 'Manipulator Path', 'NumberTitle', 'off');
    axes('Parent', fig);
    R = 5;
    r = 1.5;
    u = linspace(0, 2*pi, 60);
    v = linspace(0, 2*pi, 60);
    [U, V] = meshgrid(u, v);
    X_torus = (R + r*cos(V)).*cos(U);
    Y_torus = (R + r*cos(V)).*sin(U);
    Z_torus = r*sin(V);
    surf(X_torus, Y_torus, Z_torus, 'FaceColor', [0.8 0.8 0.8], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.25);
    hold on
    axis equal
    grid on
    view(3)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Manipulator Path')
    path_line = animatedline('Color', 'r', 'LineWidth', 2.5);
    current_pos = plot3(NaN, NaN, NaN, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
end

phi_history(:, end+1)=phi_meas;
theta1=phi_history(2, end);
theta2=phi_history(3, end);
R=5;
r=1.5;
x=(R+r*cos(theta2))*cos(theta1);
y=(R+r*cos(theta2))*sin(theta1);
z=r*sin(theta2);
addpoints(path_line, x,y,z);
set(current_pos, 'XData',x,'YData',y,'ZData',z);
drawnow limitrate
end
