function plot_results(X, U, X_ref, dt, candidate_id, constraints)
% PLOT_RESULTS - Visualize single trajectory with controls and obstacles
%
% Inputs:
%   X: state trajectory (4 x N+1)
%   U: control trajectory (2 x N)
%   X_ref: reference trajectory (4 x N+1)
%   dt: time step (s)
%   candidate_id: candidate identifier
%   constraints: struct containing obstacles (optional)

t = 0:dt:(size(X,2)-1)*dt;
t_u = 0:dt:(size(U,2)-1)*dt;

figure('Name', ['Candidate ' num2str(candidate_id)], 'Color', 'w', 'Position', [100, 100, 1200, 600]);

% XY trajectory visualization
subplot(2, 3, [1, 4]);
plot(X_ref(1,:), X_ref(2,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'reference'); hold on;
plot(X(1,:), X(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'iLQR');
plot(X(1,1), X(2,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'start');
plot(X(1,end), X(2,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'goal');

% Draw obstacles as safety ellipses
if nargin >= 6 && isfield(constraints, 'obstacles') && ~isempty(constraints.obstacles)
    theta_circle = linspace(0, 2*pi, 100);
    circle_x = cos(theta_circle);
    circle_y = sin(theta_circle);
    
    for i = 1:length(constraints.obstacles)
        obs = constraints.obstacles(i);
        
        % Transform unit circle: scale -> rotate -> translate
        scale_x = obs.a * circle_x;
        scale_y = obs.b * circle_y;
        
        R = [cos(obs.theta), -sin(obs.theta);
             sin(obs.theta),  cos(obs.theta)];
        rotated_points = R * [scale_x; scale_y];
        
        final_x = rotated_points(1, :) + obs.x;
        final_y = rotated_points(2, :) + obs.y;
        
        fill(final_x, final_y, [1, 0.6, 0.6], ...
            'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 1, ...
            'DisplayName', 'Obstacle');
        
        plot(obs.x, obs.y, 'r+', 'HandleVisibility', 'off');
    end
end

xlabel('X [m]'); ylabel('Y [m]');
title('Vehicle trajectory (X-Y)');
legend('Location', 'best');
grid on; axis equal;

% Speed tracking
subplot(2, 3, 2);
plot(t, X(4,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, X_ref(4,:), 'k--', 'LineWidth', 1);
ylabel('Speed [m/s]'); title('Speed tracking');
grid on; legend('actual', 'reference');

% Position tracking error
subplot(2, 3, 5);
error_pos = sqrt((X(1,:)-X_ref(1,:)).^2 + (X(2,:)-X_ref(2,:)).^2);
plot(t, error_pos, 'r-', 'LineWidth', 1.5);
ylabel('Position error [m]'); xlabel('Time [s]'); title('Position error');
grid on;

% Acceleration control
subplot(2, 3, 3);
plot(t_u, U(1,:), 'm-', 'LineWidth', 1.5);
ylabel('Acceleration [m/s^2]'); title('Control: acceleration');
grid on;

% Steering angle control
subplot(2, 3, 6);
plot(t_u, rad2deg(U(2,:)), 'm-', 'LineWidth', 1.5);
ylabel('Steer angle [deg]'); xlabel('Time [s]'); title('Control: steering');
grid on;
end

% simple car drawing helper
% function draw_car(x, y, phi, len, width, color, alpha_val)
% R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
% p_corners = [-len/2, len/2, len/2, -len/2;
%     -width/2, -width/2, width/2, width/2];
% p_global = R * p_corners + [x; y];
% patch(p_global(1,:), p_global(2,:), color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
% end
