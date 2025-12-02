function obs_list = update_traffic_manager(obs_list, ego_state, scenario)
% UPDATE_TRAFFIC_MANAGER
% 1. 移除超出范围 [-50, 150] 的障碍物
% 2. 随机生成新障碍物以维持数量在 [5, 10] 之间

    ego_x = ego_state(1);
    range_min = -50;
    range_max = 150;
    
    % --- 1. Despawn (移除) ---
    if ~isempty(obs_list)
        % 计算相对距离
        all_x = [obs_list.x];
        rel_x = all_x - ego_x;
        
        % 保留范围内的
        keep_mask = (rel_x >= range_min) & (rel_x <= range_max);
        obs_list = obs_list(keep_mask);
    end
    
    % --- 2. Spawn (生成) ---
    target_num = 2 + randi(2); % 2-4 辆车
    current_num = length(obs_list);
    
    % 防止死循环的安全计数器
    max_attempts = 50; 
    attempts = 0;
    
    while current_num < target_num && attempts < max_attempts
        attempts = attempts + 1;
        
        % A. 随机生成参数
        % 车道: 随机选一条
        lane_idx = randi(length(scenario.lane_centers));
        cand_y = scenario.lane_centers(lane_idx);
        % 增加一点点横向噪声 (+- 0.2m)，显得更真实
        % cand_y = cand_y + (rand() - 0.5) * 0.4;
        cand_y = cand_y + (rand() - 0.5) * 1.6;  % +-0.8m 的横向偏移
        
        % 纵向位置: 全局范围内随机
        % cand_x = ego_x + range_min + rand() * (range_max - range_min);
        cand_x = ego_x + 70 + rand() * (range_max - 70); % 更偏前方一些
        
        % 尺寸
        cand_len = 4.0 + rand() * 6.0; % [4.0, 10.0]
        cand_wid = 1.8 + rand() * 0.7; % [1.8, 2.5]
        
        % 速度: [6, 12] m/s
        if lane_idx == find(scenario.lane_centers == 0) % 中车道中等
            cand_v = 9 + rand() * 5.0; 
        elseif lane_idx == find(scenario.lane_centers == -4) % 右车道稍慢
            cand_v = 4 + rand() * 5.0; 
        elseif lane_idx == find(scenario.lane_centers == 4) % 左车道稍快
            cand_v = 12 + rand() * 5.0; 
        else
            cand_v = 4 + rand() * 8.0;
        end
        % 障碍物的速度有30%的概率为0 (静止)
        if rand() < 0.3
            cand_v = 0.0;
        end
        
        % 构造候选
        new_obs = struct('x', cand_x, 'y', cand_y, ...
                         'length', cand_len, 'width', cand_wid, ...
                         'theta', 0.0, ... % 假设都平行于路
                         'vx', cand_v, 'vy', 0.0);
                         
        % B. 重叠检测 (Safety Check)
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
    % 检查 cand_obs 是否与 obs_list 或 ego_state 重叠
    % 使用简单的矩形包围盒距离检测 (Box Distance Check)
    % 阈值: 纵向 gap 8m, 横向 gap 0.5m (防止生成得太近导致无解)
    
    SAFE_GAP_LON = 20.0; 
    SAFE_GAP_LAT = 0.5;
    
    is_colliding = false;
    
    % 1. Check against Ego
    dx_ego = abs(cand_obs.x - ego_state(1));
    dy_ego = abs(cand_obs.y - ego_state(2));
    
    % 简单近似: 两个中心点距离是否小于 (L1+L2)/2 + gap
    ego_len = 4.5; ego_wid = 2.0; % 假设自车尺寸
    
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