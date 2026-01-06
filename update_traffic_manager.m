function obs_list = update_traffic_manager(obs_list, ego_state, scenario)
% UPDATE_TRAFFIC_MANAGER
% 1. Remove obstacles outside range [-50, 150]
% 2. Randomly generate new obstacles to maintain count between [5, 10]

    ego_x = ego_state(1);
    range_min = -50;
    range_max = 150;
    
    % --- 1. Despawn (Remove) ---
    if ~isempty(obs_list)
        % Compute relative distance
        all_x = [obs_list.x];
        rel_x = all_x - ego_x;
        
        % Keep within range
        keep_mask = (rel_x >= range_min) & (rel_x <= range_max);
        obs_list = obs_list(keep_mask);
    end
    
    % --- 2. Spawn (Generate) ---
    target_num = 5; % 5 vehicles
    current_num = length(obs_list);
    
    % Safety counter to prevent infinite loop
    max_attempts = 50; 
    attempts = 0;
    
    while current_num < target_num && attempts < max_attempts
        attempts = attempts + 1;
        
        % A. Random parameter generation
        % Lane: randomly pick one
        lane_idx = randi(length(scenario.lane_centers));
        cand_y = scenario.lane_centers(lane_idx);
        % Add slight lateral noise (+- 0.2m) for realism
        % cand_y = cand_y + (rand() - 0.5) * 0.4;
        cand_y = cand_y + (rand() - 0.5) * 1.6;  % +- 0.8m lateral offset
        
        % Longitudinal position: random within global range
        % cand_x = ego_x + range_min + rand() * (range_max - range_min);
        cand_x = ego_x + 70 + rand() * (range_max - 70); % Slightly ahead
        
        % Size
        cand_len = 4.0 + rand() * 6.0; % [4.0, 10.0]
        cand_wid = 1.8 + rand() * 0.7; % [1.8, 2.5]
        
        % Velocity: [6, 12] m/s
        if lane_idx == find(scenario.lane_centers == 0) % Middle lane, medium speed
            cand_v = 9 + rand() * 5.0; 
        elseif lane_idx == find(scenario.lane_centers == -4) % Right lane, slightly slow
            cand_v = 4 + rand() * 5.0; 
        elseif lane_idx == find(scenario.lane_centers == 4) % Left lane, slightly fast
            cand_v = 12 + rand() * 5.0; 
        else
            cand_v = 4 + rand() * 8.0;
        end
        % Obstacle has 30% probability of zero velocity (static)
        if rand() < 0.3
            cand_v = 0.0;
        end
        
        % Construct candidate
        new_obs = struct('x', cand_x, 'y', cand_y, ...
                         'length', cand_len, 'width', cand_wid, ...
                         'theta', 0.0, ... % Assume all parallel to road
                         'vx', cand_v, 'vy', 0.0);
                         
        % B. Overlap detection (Safety Check)
        if ~check_overlap(new_obs, obs_list, ego_state)
            if isempty(obs_list)
                obs_list = new_obs;
            else
                obs_list(end+1) = new_obs;
            end
            current_num = current_num + 1;
        end
    end
end

function is_colliding = check_overlap(cand_obs, obs_list, ego_state)
    % Check if cand_obs overlaps with obs_list or ego_state
    % Uses simple axis-aligned bounding box distance check
    % Threshold: longitudinal gap 8m, lateral gap 0.5m (prevent generation too close)
    
    SAFE_GAP_LON = 20.0; 
    SAFE_GAP_LAT = 0.5;
    
    is_colliding = false;
    
    % 1. Check against Ego
    dx_ego = abs(cand_obs.x - ego_state(1));
    dy_ego = abs(cand_obs.y - ego_state(2));
    
    % Simple approximation: center distance < (L1+L2)/2 + gap
    ego_len = 4.5; ego_wid = 2.0; % Assume ego vehicle size
    
    if dx_ego < (cand_obs.length + ego_len)/2 + SAFE_GAP_LON && ...
       dy_ego < (cand_obs.width + ego_wid)/2 + SAFE_GAP_LAT
        is_colliding = true;
        return;
    end
    
    % 2. Check against other obstacles
    for i = 1:length(obs_list)
        other = obs_list(i);
        dx = abs(cand_obs.x - other.x);
        dy = abs(cand_obs.y - other.y);
        
        if dx < (cand_obs.length + other.length)/2 + SAFE_GAP_LON && ...
           dy < (cand_obs.width + other.width)/2 + SAFE_GAP_LAT
            is_colliding = true;
            return;
        end
    end
end