function plot_results_multimodal(results, best_idx, constraints, scenario, dt)
% PLOT_RESULTS_MULTIMODAL - Visualize multiple candidate trajectories and optimal decision
%
% Inputs:
%   results: struct array with fields .X, .U, .cand
%   best_idx: index of optimal trajectory
%   constraints: struct containing obstacles field
%   scenario: struct with lane_centers, lane_width, ego_size, v_desired
%   dt: time step (s)

% Create figure window
figure;
set(gcf, 'Name', 'Parallel Homotopic Trajectory Optimization Results', ...
    'Color', 'w', 'Position', [50, 50, 1600, 800]);

% Get time vectors
if isempty(results), return; end
N_steps = size(results(1).X, 2);
t_vec = 0:dt:(N_steps-1)*dt;
t_u_vec = 0:dt:(N_steps-2)*dt;

% XY trajectory subplot (main visualization area)
subplot(2, 2, [1, 2]); hold on; box on;
axis equal;  % Important for maintaining true aspect ratio
xlabel('Global X [m]'); ylabel('Global Y [m]');
title('Multi-Candidate Trajectories & Dynamic Environment', 'FontSize', 12);

% Draw road markings and lane boundaries
if isfield(scenario, 'lane_centers')
    centers = sort(scenario.lane_centers);
    if isfield(scenario, 'lane_width'), L_width = scenario.lane_width; else, L_width = 4.0; end
    
    y_outer_min = centers(1) - L_width / 2;
    y_outer_max = centers(end) + L_width / 2;
    
    % Road boundaries (thick solid lines)
    yline(y_outer_min, 'k-', 'LineWidth', 2.0, 'DisplayName', 'Road Boundary');
    yline(y_outer_max, 'k-', 'LineWidth', 2.0, 'HandleVisibility', 'off');
    
    % Lane dividers (dashed lines)
    for i = 1:length(centers)-1
        y_div = (centers(i) + centers(i+1)) / 2;
        yline(y_div, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    
    % Lane center lines (light dash-dot lines)
    for i = 1:length(centers)
        yline(centers(i), 'Color', [0.7 0.7 0.7], 'LineStyle', '-.', 'LineWidth', 0.5, 'HandleVisibility', 'off');
        if ~isempty(results)
            text(results(1).X(1,1)-5, centers(i), sprintf('Lane %d', i), 'Color', [0.5 0.5 0.5], 'FontSize', 8);
        end
    end
end

% Draw obstacles and their predicted trajectories
if isfield(constraints, 'obstacles') && ~isempty(constraints.obstacles)
    theta_circle = linspace(0, 2*pi, 50);
    circle_x = cos(theta_circle); circle_y = sin(theta_circle);
    
    for i = 1:length(constraints.obstacles)
        obs = constraints.obstacles(i);
        % Plot predicted trajectory
        if isfield(obs, 'prediction') && ~isempty(obs.prediction)
            plot(obs.prediction(1, :), obs.prediction(2, :), 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
        % Current obstacle position (bounding box + safety ellipse)
        scale_x = obs.a * circle_x; scale_y = obs.b * circle_y;
        R = [cos(obs.theta), -sin(obs.theta); sin(obs.theta), cos(obs.theta)];
        rotated = R * [scale_x; scale_y];
        final_x = rotated(1, :) + obs.x; final_y = rotated(2, :) + obs.y;
        draw_car_box(obs.x, obs.y, obs.theta, obs.length, obs.width, [1, 0.6, 0.6], 0.1);
        fill(final_x, final_y, [1, 0.8, 0.8], 'EdgeColor', 'r', 'FaceAlpha', 0.5, ...
            'DisplayName', sprintf('Obs %d', i));
    end
end

% Plot all candidate trajectories (gray, non-optimal)
h_cand = [];
for i = 1:length(results)
    if i == best_idx, continue; end
    if isempty(results(i).X), continue; end
    h_cand = plot(results(i).X(1, :), results(i).X(2, :), '-', ...
        'Color', [0.6, 0.6, 0.6, 0.5], 'LineWidth', 1.2);
end

% Plot optimal trajectory (highlighted in green)
if best_idx > 0 && ~isempty(results(best_idx).X)
    X_best = results(best_idx).X;
    cand_name = strrep(results(best_idx).cand.name, '_', '-');
    
    h_best = plot(X_best(1, :), X_best(2, :), 'g-', 'LineWidth', 2.5, ...
        'DisplayName', ['Selected: ' cand_name]);
    
    % Start and goal points
    plot(X_best(1,1), X_best(2,1), 'go', 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
    plot(X_best(1,end), X_best(2,end), 'gd', 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
    
    % Ego vehicle snapshots along trajectory
    draw_interval = floor(N_steps / 6);
    for k = 1:draw_interval:N_steps
        draw_car_box(X_best(1,k), X_best(2,k), X_best(3,k), ...
            scenario.ego_size.length, scenario.ego_size.width, 'g', 0.1);
    end
    
    % Focused view centered on trajectory
    x_center = (min(X_best(1,:)) + max(X_best(1,:))) / 2;
    x_range = max(X_best(1,:)) - min(X_best(1,:));
    xlim([x_center - x_range*0.6 - 10, x_center + x_range*0.6 + 20]);
    
    if exist('y_outer_min', 'var')
        ylim([y_outer_min - 3, y_outer_max + 3]);
    end
end

legend_handles = [];
legend_names = {};
if exist('h_best', 'var'), legend_handles(end+1) = h_best; legend_names{end+1} = 'Selected'; end
if ~isempty(h_cand), legend_handles(end+1) = h_cand(1); legend_names{end+1} = 'Candidates'; end
legend(legend_handles, legend_names, 'Location', 'northeast');
% grid on;

% Velocity profile subplot
subplot(2, 2, 3); hold on; grid on;
title('Velocity Profile', 'FontSize', 11);
xlabel('Time [s]'); ylabel('Velocity [m/s]');

for i = 1:length(results)
    if i == best_idx, continue; end
    if isempty(results(i).X), continue; end
    plot(t_vec, results(i).X(4, :), '-', 'Color', [0.7 0.7 0.7]);
end
if best_idx > 0
    plot(t_vec, results(best_idx).X(4, :), 'g-', 'LineWidth', 2.0);
    if isfield(results(best_idx).cand, 'v_target')
        yline(results(best_idx).cand.v_target, 'g--', 'Alpha', 0.5, 'HandleVisibility','off');
    end
end
if isfield(scenario, 'v_desired')
    yline(scenario.v_desired, 'k--', 'Target', 'HandleVisibility','off');
end

% Acceleration profile subplot
subplot(2, 2, 4); hold on; grid on;
title('Acceleration Profile', 'FontSize', 11);
xlabel('Time [s]'); ylabel('Acc [m/s^2]');

for i = 1:length(results)
    if i == best_idx, continue; end
    if isempty(results(i).U), continue; end
    plot(t_u_vec, results(i).U(1, :), '-', 'Color', [0.7 0.7 0.7]);
end
if best_idx > 0
    plot(t_u_vec, results(best_idx).U(1, :), 'g-', 'LineWidth', 2.0);
end
if isfield(constraints, 'u_max')
    yline(constraints.u_max(1), 'r:', 'LineWidth', 1.5, 'HandleVisibility','off');
    yline(constraints.u_min(1), 'r:', 'LineWidth', 1.5, 'HandleVisibility','off');
end

end

% Helper function: Draw vehicle bounding box
function draw_car_box(x, y, phi, len, width, color, alpha_val)
p_corners = [-len/2, len/2, len/2, -len/2;
    -width/2, -width/2, width/2, width/2];
R = [cos(phi), -sin(phi); sin(phi),  cos(phi)];
p_global = R * p_corners + [x; y];
patch(p_global(1,:), p_global(2,:), color, ...
    'FaceAlpha', alpha_val, 'EdgeColor', 'k', 'LineWidth', 0.5);
end