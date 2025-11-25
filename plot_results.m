% Helper: plotting
function plot_results(X, U, X_ref, dt, scenario_id, constraints)
t = 0:dt:(size(X,2)-1)*dt;
t_u = 0:dt:(size(U,2)-1)*dt;

figure('Name', ['Scenario ' num2str(scenario_id)], 'Color', 'w', 'Position', [100, 100, 1200, 600]);

% 1. trajectory (X-Y)
subplot(2, 3, [1, 4]);
plot(X_ref(1,:), X_ref(2,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'reference'); hold on;
plot(X(1,:), X(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'iLQR');
plot(X(1,1), X(2,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'start');
plot(X(1,end), X(2,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'goal');

% 绘制障碍物 (椭圆)
if nargin >= 6 && isfield(constraints, 'obstacles') && ~isempty(constraints.obstacles)
    % 定义单位圆用于变换
    theta_circle = linspace(0, 2*pi, 100);
    circle_x = cos(theta_circle);
    circle_y = sin(theta_circle);
    
    for i = 1:length(constraints.obstacles)
        obs = constraints.obstacles(i);
        
        % --- 椭圆变换逻辑 ---
        % 1. 缩放 (根据半长轴 a 和 半短轴 b)
        scale_x = obs.a * circle_x;
        scale_y = obs.b * circle_y;
        
        % 2. 旋转 (根据 heading theta)
        R = [cos(obs.theta), -sin(obs.theta);
            sin(obs.theta),  cos(obs.theta)];
        rotated_points = R * [scale_x; scale_y];
        
        % 3. 平移 (移动到中心 x, y)
        final_x = rotated_points(1, :) + obs.x;
        final_y = rotated_points(2, :) + obs.y;
        
        % 4. 绘图 (使用 fill 填充颜色)
        fill(final_x, final_y, [1, 0.6, 0.6], ...
            'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 1, ...
            'DisplayName', 'Obstacle');
        
        % 也可以画个中心点方便观察
        plot(obs.x, obs.y, 'r+', 'HandleVisibility', 'off');
    end
end

xlabel('X [m]'); ylabel('Y [m]');
title('Vehicle trajectory (X-Y)');
legend('Location', 'best');
grid on; axis equal;

% 2. speed tracking
subplot(2, 3, 2);
plot(t, X(4,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, X_ref(4,:), 'k--', 'LineWidth', 1);
ylabel('Speed [m/s]'); title('Speed tracking');
grid on; legend('actual', 'reference');

subplot(2, 3, 5);
% lateral error (Euclidean)
error_pos = sqrt((X(1,:)-X_ref(1,:)).^2 + (X(2,:)-X_ref(2,:)).^2);
plot(t, error_pos, 'r-', 'LineWidth', 1.5);
ylabel('Position error [m]'); xlabel('Time [s]'); title('Position error');
grid on;

% 3. controls
subplot(2, 3, 3);
plot(t_u, U(1,:), 'm-', 'LineWidth', 1.5);
ylabel('Acceleration [m/s^2]'); title('Control: acceleration');
grid on;

subplot(2, 3, 6);
plot(t_u, rad2deg(U(2,:)), 'm-', 'LineWidth', 1.5);
ylabel('Steer angle [deg]'); xlabel('Time [s]'); title('Control: steer');
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
