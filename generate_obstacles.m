function obstacles = generate_obstacles(raw_obs_list, ego_size, dt, N)
% GENERATE_OBSTACLES - Inflate rectangular obstacles and predict trajectories
% 
% This function:
%   1. Inflates rectangular obstacles into safety ellipses (Minkowski sum approximation)
%   2. Predicts obstacle trajectories over N steps using constant velocity model
%
% Inputs:
%   raw_obs_list: struct array with fields [x, y, length, width, theta, vx, vy]
%   ego_size: struct with fields [length, width]
%   dt: time step (seconds)
%   N: number of prediction steps
%
% Output:
%   obstacles: struct array with fields [a, b, prediction, ...]
%     - a, b: ellipse semi-axes
%     - prediction: 3xN matrix [x; y; theta] for each prediction step

obstacles = [];
if isempty(raw_obs_list)
    return;
end

for i = 1:length(raw_obs_list)
    raw = raw_obs_list(i);
    obs = raw; % Copy base fields
    
    % Inflate obstacle using Minkowski sum approximation
    % Ellipse semi-major axis: a = (obstacle_length + ego_length)/2 + safety_margin_lon
    % Ellipse semi-minor axis: b = (obstacle_width + ego_width)/2 + safety_margin_lat
    safe_margin_lon = 3.0; % Longitudinal safety margin (m)
    safe_margin_lat = 0.8; % Lateral safety margin (m)
    obs.a = (raw.length + ego_size.length) / 2.0 + safe_margin_lon;
    obs.b = (raw.width  + ego_size.width)  / 2.0 + safe_margin_lat;
    
    % Trajectory prediction using constant velocity model
    % Generate 3xN matrix: [x; y; theta] for each prediction step (t = 0 to (N-1)*dt)

    if (norm([raw.vx; raw.vy]) < 1e-3)
        % Static obstacle - no prediction needed
        obs.prediction = [];
        obstacles = [obstacles, obs];
        continue;
    end
    
    % Time sequence: [0, dt, 2*dt, ..., (N-1)*dt]
    t_seq = (0:N-1) * dt;
    
    % Linear kinematic model: position(t) = position_0 + velocity * t
    pred_x = raw.x + raw.vx * t_seq;
    pred_y = raw.y + raw.vy * t_seq;
    pred_theta = repmat(raw.theta, 1, N); % Heading remains constant
    
    obs.prediction = [pred_x; pred_y; pred_theta];
    
    obstacles = [obstacles, obs];
end
end
