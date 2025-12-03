function [scenario, constraints, x0, raw_obs_list] = build_simulation_scenario(dt, N)
% BUILD_SIMULATION_SCENARIO Create a three-lane scenario with dynamic and static obstacles

% --- 1. Road geometry (three lanes, 4m wide) ---
% Lane 1 (Right): y = -4
% Lane 2 (Center): y = 0
% Lane 3 (Left): y = 4
scenario.lane_centers = [-4.0, 0.0, 4.0];
scenario.lane_width = 4.0;

% --- 2. Ego vehicle initial state ---
% Positioned in the center lane, speed 10 m/s
x0 = [0.0; 0.0; 0.0; 10.0];

% Ego vehicle dimensions
ego_size.length = 4.5;
ego_size.width = 2.0;
scenario.ego_size = ego_size;

% --- 3. Road boundary constraints (for vehicle centroid) ---
% Physical edges: leftmost lane center + half width, rightmost lane center - half width
road_edge_left  = max(scenario.lane_centers) + scenario.lane_width/2; % +6.0
road_edge_right = min(scenario.lane_centers) - scenario.lane_width/2; % -6.0

% Allowed range for centroid: physical edges shrunk by half vehicle width + safety margin
margin_safety = 0.2; % 0.2m additional buffer
y_max = road_edge_left  - ego_size.width/2 - margin_safety;
y_min = road_edge_right + ego_size.width/2 + margin_safety;

constraints.road_bounds = [y_min, y_max]; % [-4.8, 4.8]

% --- 4. Obstacle generation ---
obs1 = struct('x', 40.0, 'y', 0.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 6.0, 'vy', 0.0);
obs2 = struct('x', 5.0, 'y', 4.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 8.0, 'vy', 0.0);
obs3 = struct('x', 60.0, 'y', -4.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 0.0, 'vy', 0.0);

raw_obs_list = [obs1, obs2, obs3];
constraints.obstacles = generate_obstacles(raw_obs_list, ego_size, dt, N);

% --- 5. Physical constraints ---
constraints.u_min = [-2.0; -0.5]; % Minimum control inputs
constraints.u_max = [ 2.0;  0.5]; % Maximum control inputs

% Scenario metadata
scenario.v_desired = 20.0; % Desired velocity
end