function candidates = generate_candidates_structured(x0, scenario, constraints, N, dt)
% GENERATE_CANDIDATES_STRUCTURED Generate multimodal candidate trajectories
% using target_velocity/position_solver
%
% Inputs:
%   x0: ego state [x; y; theta; v; a; kappa] (optional: a defaults to 0)
%   scenario: contains lane_centers, v_desired
%   constraints: contains obstacles, u_max, u_min, limits (jerk limits)
%   N, dt: planning horizon and time step

% --- 1. Parse inputs ---
p0_x = x0(1);
p0_y = x0(2);
v0 = x0(4);
if length(x0) >= 5, a0 = x0(5); else, a0 = 0; end

% Physical limits (used by solvers)
limits.a_max = constraints.u_max(1);
limits.a_min = constraints.u_min(1);
limits.j_max = 1.0; % m/s^3
limits.j_min = -1.0;

v_max = 20.0;
v_min = 0.0;
v_desired = scenario.v_desired;

candidates = [];
cand_id = 1;

% --- 2. Iterate over lateral intentions (lanes) ---
% Find current lane index
[~, curr_lane_idx] = min(abs(scenario.lane_centers - p0_y));

% Define search directions: [current, left, right]
search_offsets = [0, 1, -1];

for offset = search_offsets
    target_lane_idx = curr_lane_idx + offset;

    % Check if lane exists
    if target_lane_idx < 1 || target_lane_idx > length(scenario.lane_centers)
        continue;
    end

    target_y = scenario.lane_centers(target_lane_idx);

    % Define lateral action label
    if offset == 0, lat_action = 'Keep';
    elseif offset == 1, lat_action = 'Left';
    else, lat_action = 'Right';
    end

    % --- 3. Perception: find leading obstacle in target lane ---
    [has_lead, lead_obs] = find_leading_obstacle(p0_x, target_y, constraints.obstacles);

    % ===========================================================
    % Longitudinal mode A: reach desired speed (Cruise/Overtake)
    % ===========================================================
    % Call velocity solver
    P_cruise = target_velocity_solver(v0, a0, v_desired, limits);
    % plot_target_velocity_result(P_cruise, v0, a0, p0_x, limits, N*dt);

    % Generate discretized trajectory
    [traj_cruise, acc_cruise] = discretize_trajectory(P_cruise, 'velocity', x0, target_y, N, dt);
    % figure;
    % subplot(2,1,1);
    % plot(traj_cruise(1, :), traj_cruise(2, :), 'LineWidth', 1.0);
    % legend('Cruise Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on;
    % subplot(2,1,2);
    % plot(0:dt:dt*N, traj_cruise(4, :), 'LineWidth', 1.0);
    % legend('Velocity Profile'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;
    % close all;

    % Add candidate
    candidates = add_candidate(candidates, cand_id, ...
        [lat_action, '_Cruise'], traj_cruise, acc_cruise, v_desired, target_lane_idx);
    cand_id = cand_id + 1;

    % ===========================================================
    % Longitudinal mode B: adapt to leading vehicle (Follow/Yield/Stop)
    % ===========================================================
    if has_lead
        % Get leading vehicle info
        v_lead = lead_obs.vx;
        p_lead = lead_obs.x;
        safe_dist = 10.0 + v_lead * 2.0; % Simple safety distance model

        target_v = v_lead;
        stable_following_time = 2.0; % seconds for stable following
        target_pos = p_lead + stable_following_time * v_lead - safe_dist;

        if target_pos > p0_x
            [Pa, Pb, tpb, tc, ~] = target_position_solver(p0_x, v0, a0, target_pos, target_v, limits, v_max, v_min);

            % target_position_results.Pa = Pa;
            % target_position_results.Pb = Pb;
            % target_position_results.tpb = tpb;
            % target_position_results.tc = tc;
            % target_position_results.T_total = t_total;
            % plot_target_position_result(target_position_results, p0_x, v0, a0, limits, dt, 15, target_pos);
            % close all;

            % Package position solver output
            solver_out.Pa = Pa;
            solver_out.Pb = Pb;
            solver_out.tpb = tpb;
            solver_out.tc = tc;

            % Generate trajectory
            [traj_stop, acc_stop] = discretize_trajectory(solver_out, 'position', x0, target_y, N, dt);
            % figure;
            % subplot(2,1,1);
            % plot(traj_stop(1, :), traj_stop(2, :), 'LineWidth', 1.0);
            % legend('Cruise Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on;
            % subplot(2,1,2);
            % plot(0:dt:dt*N, traj_stop(4, :), 'LineWidth', 1.0);
            % legend('Velocity Profile'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;
            % close all;

            candidates = add_candidate(candidates, cand_id, ...
                [lat_action, '_Stop'], traj_stop, acc_stop, 0, target_lane_idx);
            cand_id = cand_id + 1;
        end

    end
end
end

% =========================================================
% Helper functions
% =========================================================

function [found, obs] = find_leading_obstacle(ego_x, lane_y, obstacles)
% Find nearest obstacle ahead in specified lane
min_dist = inf;
found = false;
obs = [];
lane_threshold = 1.5; % Half lane width to check if obstacle is in lane

for i = 1:length(obstacles)
    o = obstacles(i);
    % Check if ahead
    if o.x > ego_x
        % Check lateral distance
        if abs(o.y - lane_y) < lane_threshold
            dist = o.x - ego_x;
            if dist < min_dist
                min_dist = dist;
                obs = o;
                found = true;
            end
        end
    end
end
end

function new_list = add_candidate(list, id, name, traj, acc, v_tgt, lane_id)
c.id = id;
c.name = name;
c.x_ref = traj;
c.v_target = v_tgt;
c.acc = acc;
c.target_lane_id = lane_id;

if isempty(list)
    new_list = c;
else
    new_list = [list, c];
end
end

function [traj, acc] = discretize_trajectory(P_struct, mode, x0, target_y, N, dt)
% Discretize trajectory from solver output (velocity or position mode)
% P_struct: velocity solver output P, or position solver output with Pa, Pb, tpb, tc

traj = zeros(4, N+1);
acc = zeros(1, N+1);

p0_x = x0(1);
p0_y = x0(2);
theta0 = x0(3);
v0 = x0(4);
if length(x0)>=5, a0=x0(5); else, a0=0; end

% Assume lane change duration
T_lane_change = 4.0;

for k = 1:N+1
    t = (k-1) * dt;

    % --- 1. Longitudinal computation ---
    if strcmp(mode, 'velocity')
        % Call get_state_at_t directly
        [p_val, v_val, a_val] = get_state_at_t(v0, a0, p0_x, P_struct, t);
    else % mode == 'position'
        % Use evaluate_position wrapper for piecewise trajectory
        [p_val, v_val, a_val] = evaluate_position(P_struct, v0, a0, p0_x, t);
    end

    % --- 2. Lateral computation (smooth interpolation) ---
    if t < T_lane_change
        ratio = t / T_lane_change;
        s = 10*ratio^3 - 15*ratio^4 + 6*ratio^5; % Quintic polynomial
        y_val = p0_y + (target_y - p0_y) * s;
    else
        y_val = target_y;
    end

    % --- 3. Heading angle approximation ---
    if k > 1
        dx = p_val - traj(1, k-1);
        dy = y_val - traj(2, k-1);
        % Avoid discontinuity when stationary
        if sqrt(dx^2+dy^2) > 1e-3
            theta_val = atan2(dy, dx);
        else
            theta_val = traj(3, k-1);
        end
    else
        theta_val = theta0;
    end

    traj(:, k) = [p_val; y_val; theta_val; v_val];
    acc(k) = a_val;
end
end