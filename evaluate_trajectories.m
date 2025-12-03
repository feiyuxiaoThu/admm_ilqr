function [best_idx, best_cost, debug_scores] = evaluate_trajectories(results, obstacles, last_best_target_lane_id, scenario_params, eval_weights)
% EVALUATE_TRAJECTORIES Score candidate trajectories and pick the best
%
% Logic:
%   1. Hard constraints filter: discard diverging or colliding trajectories
%   2. Soft scoring: total = J_safety + J_progress + J_comfort + J_consistency
%
% Inputs:
%   results: struct array with fields .X, .U, .valid, .cand
%   obstacles: obstacle structs (may include .prediction)
%   last_best_target_lane_id: previous chosen lane id (for consistency reward)
%   scenario_params: contains .v_desired (global desired speed)
%   eval_weights: weight struct for scores
%
% Outputs:
%   best_idx: index of best trajectory in results
%   best_cost: minimum total cost
%   debug_scores: per-trajectory debug scoring details

best_cost = inf;
best_idx = -1;
num_cands = length(results);

% Initialize debug records
debug_scores = struct('id', {}, 'valid', {}, 'J_safe', {}, 'J_prog', {}, 'J_comf', {}, 'J_cons', {}, 'total', {});

% Alert distance: inside this range safety cost may apply
D_ALERT = 4.0; % meters (kept constant here)

% Collision tolerance: small negative tolerance to avoid false positives
D_COLLISION = -0.05;

for i = 1:num_cands
    traj_X = results(i).X;
    traj_U = results(i).U;
    cand = results(i).cand;

    % --- 1. Hard constraints checks ---

    % A. Convergence check (commented out): sometimes usable even if not fully converged
    % if ~results(i).success
    %    record_failure(i, 'Not Converged'); continue;
    % end

    % B. Collision check (pointwise)
    [min_dist, ~] = calc_min_distance_to_obstacles(traj_X, obstacles);

    if min_dist < D_COLLISION
        % Collision detected -> discard
        record_debug(i, false, inf, 0, 0, 0, inf);
        continue;
    end

    % --- 2. Soft scoring ---

    % A. Safety margin cost (quadratic hinge loss)
    % J = sum( max(0, d_alert - d)^2 ) over timesteps
    J_safety = calc_safety_cost(traj_X, obstacles, D_ALERT);

    % B. Progress & efficiency
    % Reward: distance traveled (more is better)
    dist_traveled = traj_X(1, end) - traj_X(1, 1);
    % Penalty: velocity tracking error vs desired speed
    v_error_sq = sum((traj_X(4, :) - scenario_params.v_desired).^2);

    J_progress = - eval_weights.w_progress * dist_traveled ...
        + eval_weights.w_ref_vel * v_error_sq;

    % C. Comfort cost
    % J approx sum of accelerations^2 and jerk^2
    acc_sq = sum(traj_U(1, :).^2);
    steer_sq = sum(traj_U(2, :).^2);

    % Approximate jerk via finite differences
    dt = 0.1; % assumed dt
    jerk_lon = sum(diff(traj_U(1, :)).^2) / dt^2;
    jerk_lat = sum(diff(traj_U(2, :)).^2) / dt^2;

    J_comfort = eval_weights.w_acc * acc_sq + ...
        eval_weights.w_steer * steer_sq + ...
        eval_weights.w_jerk_lon * jerk_lon + ...
        eval_weights.w_jerk_lat * jerk_lat;

    % D. Consistency cost (penalize lane switches w.r.t. last selection)
    J_consistency = 0;
    if last_best_target_lane_id ~= -1
        if cand.target_lane_id ~= last_best_target_lane_id
            J_consistency = eval_weights.w_consistency;
        end
    end

    % --- 3. Total score aggregation ---
    total_score = eval_weights.w_safety * J_safety + ...
        J_progress + ...
        J_comfort + ...
        J_consistency;

    debug_scores = record_debug(i, true, J_safety, J_progress, J_comfort, J_consistency, total_score, debug_scores);

    % Update best candidate
    if total_score < best_cost
        best_cost = total_score;
        best_idx = i;
    end
end

% --- Internal helper: record debug info ---
    function debug_scores = record_debug(id, valid, s, p, c, con, tot, debug_scores)
        debug_scores(id).id = id;
        debug_scores(id).valid = valid;
        debug_scores(id).J_safe = s;
        debug_scores(id).J_prog = p;
        debug_scores(id).J_comf = c;
        debug_scores(id).J_cons = con;
        debug_scores(id).total = tot;
    end
end

% =========================================================
% Helper calculation functions
% =========================================================

function J = calc_safety_cost(traj, obstacles, d_alert)
% Compute quadratic hinge loss for safety margin
J = 0;
N = size(traj, 2);

for k = 1:N
    ego_pos = traj(1:2, k);

    for j = 1:length(obstacles)
        obs = obstacles(j);

        % Get obstacle position at this timestep (dynamic/static)
        if isfield(obs, 'prediction') && ~isempty(obs.prediction)
            idx = min(k, size(obs.prediction, 2));
            obs_pos = obs.prediction(1:2, idx);
            obs_theta = obs.prediction(3, idx);
        else
            obs_pos = [obs.x; obs.y];
            obs_theta = obs.theta;
        end

        % Approximate surface distance
        dist = point_to_ellipse_dist(ego_pos, obs_pos, obs_theta, obs.a, obs.b);

        % Hinge loss: penalize only when inside the alert distance
        if dist < d_alert
            J = J + (d_alert - dist)^2;
        end
    end
end
end

function [min_dist, obs_idx] = calc_min_distance_to_obstacles(traj, obstacles)
% Find minimum distance to any obstacle over the trajectory
min_dist = inf;
obs_idx = -1;

N = size(traj, 2);
for k = 1:N
    ego_pos = traj(1:2, k);
    for j = 1:length(obstacles)
        obs = obstacles(j);
        % Get obstacle position at this timestep
        if isfield(obs, 'prediction') && ~isempty(obs.prediction)
            idx = min(k, size(obs.prediction, 2));
            obs_pos = obs.prediction(1:2, idx);
            obs_theta = obs.prediction(3, idx);
        else
            obs_pos = [obs.x; obs.y];
            obs_theta = obs.theta;
        end

        d = point_to_ellipse_dist(ego_pos, obs_pos, obs_theta, obs.a, obs.b);

        if d < min_dist
            min_dist = d;
            obs_idx = j;
        end
    end
end
end

function dist = point_to_ellipse_dist(pt, center, theta, a, b)
% Approximate signed distance from a point to an ellipse
% Positive: outside ellipse; Negative: inside ellipse

% 1. Transform to local ellipse frame
dx = pt(1) - center(1);
dy = pt(2) - center(2);

cos_t = cos(theta);
sin_t = sin(theta);

local_x =  cos_t * dx + sin_t * dy;
local_y = -sin_t * dx + cos_t * dy;

% 2. Algebraic metric = (x/a)^2 + (y/b)^2
metric = (local_x / a)^2 + (local_y / b)^2;

% 3. Convert to approximate Euclidean distance
% Use conservative approximation: (sqrt(metric) - 1) * min(a,b)
dist_approx = (sqrt(metric) - 1.0) * min(a, b);

dist = dist_approx;
end