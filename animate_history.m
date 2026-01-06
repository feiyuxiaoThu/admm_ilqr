function animate_history(save_video)
% ANIMATE_HISTORY Render closed-loop simulation results as animation
%
% Input:
%   save_video: bool, save as MP4 (default false)

if nargin < 1, save_video = false; end

% Load data
if ~isfile('sim_results.mat')
    error('Result file not found. Run simulation first.');
end
load('sim_results.mat', 'sim_log', 'scenario');

N_sim = length(sim_log);

% Prepare figure window
figure('Name', 'Closed-Loop Simulation Playback', 'Color', 'w', ...
    'Position', [100, 100, 1200, 600]);

% Layout: main plot above, dashboard below
ax_main = subplot(1, 2, [1, 2]);
hold(ax_main, 'on'); axis(ax_main, 'equal'); box(ax_main, 'on');
xlabel('Global X [m]'); ylabel('Global Y [m]');
title(ax_main, 'Autonomous Driving Simulation');

% Draw static scene (lane markings)
draw_road_network(ax_main, scenario);

% Initialize handles for dynamic objects
h_ego_body = [];      % Ego vehicle patch
h_obs_bodies = [];    % Obstacle patches array
h_obs_preds = [];     % Obstacle prediction lines array
h_cands = [];         % Candidate trajectories array
h_plan = [];          % Optimal planned trajectory line
h_title_text = [];    % Status text

% Video recording setup
% save to mp4
if save_video
    v_writer = VideoWriter('simulation_video.mp4', 'MPEG-4');
    v_writer.FrameRate = 20;
    open(v_writer);
end

% save to gif
% gif_filename = 'simulation_video.gif';

% Animation loop
for k = 1:N_sim
    frame_data = sim_log(k);

    % Update ego vehicle
    x_ego = frame_data.ego_state;

    if isempty(h_ego_body)
        % Initialize
        h_ego_body = draw_car_patch(ax_main, x_ego, scenario.ego_size, 'g', 0.8);
    else
        % Update
        update_car_patch(h_ego_body, x_ego, scenario.ego_size);
    end

    % Update obstacles
    obs_list = frame_data.obstacles;
    num_obs = length(obs_list);

    % Expand handle array if obstacle count changes
    if length(h_obs_bodies) < num_obs
        for j = length(h_obs_bodies)+1 : num_obs
            h_obs_bodies(j) = draw_car_patch(ax_main, [0;0;0;0], struct('length',4.5,'width',2), [1 0.7 0.7], 0.6);
            h_obs_preds(j) = plot(ax_main, 0, 0, 'r:', 'LineWidth', 1.0);
        end
    end

    for j = 1:num_obs
        obs = obs_list(j);
        % Construct state vector [x, y, theta, v]
        obs_state = [obs.x; obs.y; obs.theta; 0];
        obs_size = struct('length', obs.length, 'width', obs.width);
        update_car_patch(h_obs_bodies(j), obs_state, obs_size);

        % Update prediction line
        if isfield(obs, 'prediction') && ~isempty(obs.prediction)
            set(h_obs_preds(j), 'XData', obs.prediction(1,:), 'YData', obs.prediction(2,:));
        end
    end

    % Update planning results
    plan = frame_data.plan_info;

    % Candidate trajectories (gray)
    results = plan.results;
    num_cands = length(results);

    % Manage handles
    if length(h_cands) < num_cands
        for j = length(h_cands)+1 : num_cands
            h_cands(j) = plot(ax_main, 0, 0, '-', 'Color', [0.6 0.6 0.6, 0.4], 'LineWidth', 1.0);
        end
    end

    for j = 1:length(h_cands)
        if j <= num_cands && ~isempty(results(j).X)
            set(h_cands(j), 'XData', results(j).X(1,:), 'YData', results(j).X(2,:), 'Visible', 'on');
        else
            set(h_cands(j), 'Visible', 'off');
        end
    end

    % Optimal trajectory (green thick line)
    best_idx = plan.best_idx;
    if best_idx ~= -1
        best_traj = results(best_idx).X;
        if isempty(h_plan)
            h_plan = plot(ax_main, best_traj(1,:), best_traj(2,:), 'g-', 'LineWidth', 2.5);
        else
            set(h_plan, 'XData', best_traj(1,:), 'YData', best_traj(2,:), 'Visible', 'on');
        end
    else
        set(h_plan, 'Visible', 'off');
    end

    % Camera follow: keep ego vehicle center-left
    view_range_x = [-20, 120];
    view_range_y = [-10, 10];

    xlim(ax_main, x_ego(1) + view_range_x);
    ylim(ax_main, 0 + view_range_y);

    % Update status text
    info_str = {
        sprintf('Time: %.1f s', frame_data.time),
        sprintf('Speed: %.1f m/s', x_ego(4)),
        sprintf('Intent: %s', strrep(plan.best_cand_name, '_', '\_')),
        sprintf('Cost: %.2f', plan.best_score)
        };

    if isempty(h_title_text)
        title(ax_main, info_str, 'FontSize', 12, 'Interpreter', 'tex');
    else
        title(ax_main, info_str);
    end

    drawnow limitrate;

    if save_video
        % save to mp4
        frame = getframe(gcf);
        writeVideo(v_writer, frame);

        % save to gif
        % frame = getframe(gcf); 
        % im = frame2im(frame); 
        % [imind, cm] = rgb2ind(im, 256); 
        % if k == 1
        %     imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05); 
        % else
        %     imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05); 
        % end
    else
        pause(0.04);
    end
end

if save_video
    % save to mp4
    close(v_writer); 

    fprintf('Video saved to simulation_video\n');
end
end

% ========== Plotting Helper Functions ==========

function draw_road_network(ax, scenario)
% Draw lane markings
centers = scenario.lane_centers;
width = 4.0;
if isfield(scenario, 'lane_width'), width = scenario.lane_width; end

x_range = [-100, 2000];

% Road boundaries
y_min = min(centers) - width/2;
y_max = max(centers) + width/2;
plot(ax, x_range, [y_min, y_min], 'k-', 'LineWidth', 2);
plot(ax, x_range, [y_max, y_max], 'k-', 'LineWidth', 2);

% Lane dividers
for i = 1:length(centers)-1
    y_div = (centers(i) + centers(i+1)) / 2;
    plot(ax, x_range, [y_div, y_div], 'k--', 'LineWidth', 1);
end
end

function h_patch = draw_car_patch(ax, x_state, size_struct, color, alpha)
% Create vehicle shape
len = size_struct.length;
wid = size_struct.width;

% Initial local coordinates
p_corners = [-len/2, len/2, len/2, -len/2;
    -wid/2, -wid/2, wid/2, wid/2];

h_patch = patch(ax, p_corners(1,:), p_corners(2,:), color, ...
    'FaceAlpha', alpha, 'EdgeColor', 'k');

update_car_patch(h_patch, x_state, size_struct);
end

function update_car_patch(h_patch, x_state, size_struct)
% Update vehicle position and orientation
cx = x_state(1); cy = x_state(2); phi = x_state(3);
len = size_struct.length; wid = size_struct.width;

p_corners = [-len/2, len/2, len/2, -len/2;
    -wid/2, -wid/2, wid/2, wid/2];

R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
p_global = R * p_corners + [cx; cy];

set(h_patch, 'XData', p_global(1,:), 'YData', p_global(2,:));
end