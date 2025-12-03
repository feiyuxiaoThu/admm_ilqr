function plot_iteration(fig_handle, dt, debug_info, constraints, x_ref_traj, plot_ilqr_iter, ego_size)
% PLOT_ITERATION - Animate stored iteration histories (ADMM + inner iLQR)
%
% Visualizes XY trajectories and controls for each saved iLQR/ADMM iteration.
% Displays ego vehicle, obstacles, and convergence metrics.
%
% Inputs:
%   fig_handle: figure handle or empty
%   dt: time step (s)
%   debug_info: struct with X_history, U_history, residual_history, cost_history
%   constraints: struct with obstacles field
%   x_ref_traj: reference trajectory (4 x N+1)
%   plot_ilqr_iter: boolean, whether to animate each iLQR iteration
%   ego_size: struct with length, width fields

% Set up figure with large size
fig_pos = [100, 50, 1400, 800];
if ~exist('fig_handle','var') || ~ishandle(fig_handle)
    fig_handle = figure('Name', 'iLQR iteration history', 'NumberTitle', 'off', 'Units', 'pixels', 'Position', fig_pos);
else
    try
        set(fig_handle, 'Units', 'pixels', 'Position', fig_pos);
        figure(fig_handle);
    catch
        fig_handle = figure('Name', 'iLQR iteration history', 'NumberTitle', 'off', 'Units', 'pixels', 'Position', fig_pos);
    end
end

Np1 = size(debug_info.X_history{1}, 2);
niters_admm = size(debug_info.X_history, 2);

% Time vectors for control visualization
t_u = (0:(Np1-2)) * dt;

% Pre-compute unit circle for ellipse drawing
theta_circle = linspace(0, 2*pi, 60);
unit_circle_x = cos(theta_circle);
unit_circle_y = sin(theta_circle);

% Animation pause times
if plot_ilqr_iter
    ilqr_pause_time = 0.1;
    admm_pause_time = 0.2;
else
    ilqr_pause_time = 0.0;
    admm_pause_time = 0.05;
end

% Animate ADMM iterations
for k = 1 : niters_admm
    X_hist = debug_info.X_history{k};
    U_hist = debug_info.U_history{k};
    
    niters_ilqr = size(X_hist, 3);
    colors = parula(max(3, niters_ilqr));
    
    % Animate iLQR iterations (or show only final)
    for ii = 1 : niters_ilqr
        if ~plot_ilqr_iter && ii < niters_ilqr
            continue;
        end
        
        clf(fig_handle);
        
        % XY trajectory subplot (left)
        subplot(2,4,[1,2,5,6]); hold on; grid on; axis equal;
        Xk = X_hist(:, :, ii);
        if ii < niters_ilqr
            plot(Xk(1,:), Xk(2,:), '-', 'Color', colors(ii,:), 'LineWidth', 1);
        else
            plot(Xk(1,:), Xk(2,:), 'g-', 'LineWidth', 1.5);
        end
        plot(x_ref_traj(1,:), x_ref_traj(2,:), 'b--', 'LineWidth', 1);
        plot(Xk(1,1), Xk(2,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'start');
        plot(x_ref_traj(1,end), x_ref_traj(2,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'goal');
        
        % Draw ego vehicle at start
        draw_box(Xk(1,1), Xk(2,1), Xk(3,1), ego_size.length, ego_size.width, [0, 1, 0], 0.4);
        text(Xk(1,1), Xk(2,1), '0', 'Color', 'k', 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        
        % Draw ego vehicle snapshots every 1 second
        steps_per_sec = round(1.0 / dt);
        snapshot_indices = (steps_per_sec + 1) : steps_per_sec : size(Xk, 2);
        
        for k_idx = snapshot_indices
            ego_x = Xk(1, k_idx);
            ego_y = Xk(2, k_idx);
            draw_box_outline(ego_x, ego_y, Xk(3, k_idx), ego_size.length, ego_size.width, [0, 0.6, 0]);
            t_val = (k_idx - 1) * dt;
            text(ego_x, ego_y, sprintf('%.0f', t_val), 'Color', [0, 0.4, 0], 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        end
        
        % Draw obstacles
        if isfield(constraints, 'obstacles')
            for io = 1:length(constraints.obstacles)
                obs = constraints.obstacles(io);
                
                % Current position (t=0) - filled box
                draw_box(obs.x, obs.y, obs.theta, obs.length, obs.width, [0.3 0.3 0.3], 0.3);
                
                is_dynamic = norm([obs.vx, obs.vy]) > 0.1;
                
                if is_dynamic
                    % Dynamic obstacle: plot predicted trajectory and future positions
                    plot(obs.prediction(1, :), obs.prediction(2, :), 'r:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
                    
                    % Future snapshots (ensure valid prediction indices)
                    max_pred_len = size(obs.prediction, 2);
                    valid_indices = snapshot_indices(snapshot_indices <= max_pred_len);
                    
                    for k_idx = valid_indices
                        curr_x = obs.prediction(1, k_idx);
                        curr_y = obs.prediction(2, k_idx);
                        
                        % Future bounding box outline
                        draw_box_outline(curr_x, curr_y, obs.theta, obs.length, obs.width, [0.5 0.5 0.5]);
                        
                        % Future safety ellipse
                        draw_ellipse(curr_x, curr_y, obs.theta, obs.a, obs.b, unit_circle_x, unit_circle_y, 'r:');
                        
                        t_val = (k_idx - 1) * dt;
                        text(curr_x, curr_y, sprintf('%.0f', t_val), 'Color', 'k', 'FontSize', 8, 'HorizontalAlignment', 'center');
                    end
                else
                    % Static obstacle: plot safety ellipse only
                    draw_ellipse(obs.x, obs.y, obs.theta, obs.a, obs.b, unit_circle_x, unit_circle_y, 'r--');
                end
            end
        end
        
        legend('Optimized Trajectory', 'Reference Trajectory', 'Start', 'Goal', 'Location', 'best');
        title(sprintf('XY (iLQR %d/%d, ADMM %d/%d)', ii, niters_ilqr, k, niters_admm));
        xlabel('X'); ylabel('Y');
        
        % Acceleration subplot
        subplot(2,4,3); hold on; grid on;
        Uk = squeeze(U_hist(1, :, ii));
        if ii < niters_ilqr
            plot(t_u, Uk, '-', 'Color', colors(ii,:));
        else
            plot(t_u, Uk, 'g-', 'LineWidth', 1.0);
        end
        title('Acceleration over time'); xlabel('time(s)'); ylabel('acc(m/s^2)');
        
        % Steering subplot
        subplot(2,4,7); hold on; grid on;
        Uk = squeeze(U_hist(2, :, ii)) * (180/pi);
        if ii < niters_ilqr
            plot(t_u, Uk, '-', 'Color', colors(ii,:));
        else
            plot(t_u, Uk, 'g-', 'LineWidth', 1.0);
        end
        title('Steer angle over time'); xlabel('time(s)'); ylabel('steer(degree)');
        
        % Primal residual convergence
        subplot(2,4,4); hold on; grid on; xlim([1, max(2, niters_admm)]);
        if ii == niters_ilqr
            if isfield(debug_info, 'residual_history') && length(debug_info.residual_history) == niters_admm
                res_vals = debug_info.residual_history(1 : k);
                plot(1 : 1: k, res_vals, 'b-o');
                title('Primal Residual over ADMM iterations'); xlabel('ADMM iteration'); ylabel('Primal Residual');
            end
        end
        
        % Cost convergence
        subplot(2,4,8); hold on; grid on; xlim([1, max(2, niters_admm)]);
        if ii == niters_ilqr
            if isfield(debug_info, 'cost_history') && length(debug_info.cost_history) == niters_admm
                cost_vals = debug_info.cost_history(1 : k);
                plot(1 : 1: k, cost_vals, 'm-s');
                title('Cost over ADMM iterations'); xlabel('ADMM iteration'); ylabel('Cost');
            end
        end
        
        drawnow;
        % Pause for animation
        if ii < niters_ilqr
            pause(ilqr_pause_time);
        else
            pause(admm_pause_time);
        end
    end
end
end

% --- Helper drawing functions ---

% Draw filled rectangle (solid)
function draw_box(x, y, theta, L, W, color, alpha_val)
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
corners = [-L/2, L/2, L/2, -L/2; -W/2, -W/2, W/2, W/2];
pts = R * corners + [x; y];
fill(pts(1,:), pts(2,:), color, 'FaceAlpha', alpha_val, 'EdgeColor', 'k', 'LineWidth', 0.5, 'HandleVisibility', 'off');
end

% Draw rectangle outline (hollow)
function draw_box_outline(x, y, theta, L, W, color)
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
corners = [-L/2, L/2, L/2, -L/2, -L/2; -W/2, -W/2, W/2, W/2, -W/2];
pts = R * corners + [x; y];
plot(pts(1,:), pts(2,:), '--', 'Color', color, 'LineWidth', 0.8, 'HandleVisibility', 'off');
end

% Draw ellipse (rotated)
function draw_ellipse(x, y, theta, a, b, ux, uy, line_style)
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
pts = R * [a*ux; b*uy] + [x; y];
plot(pts(1,:), pts(2,:), line_style, 'LineWidth', 0.8, 'HandleVisibility', 'off');
end