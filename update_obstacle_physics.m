function updated_list = update_obstacle_physics(obs_list, dt)
    % 更新真实障碍物状态 (Ground Truth Physics)
    % 目前假设简单的匀速直线运动，如果要测试复杂场景（如前车突然减速），在这里修改
    
    updated_list = obs_list;
    for i = 1:length(obs_list)
        % 简单的运动学更新
        obs = obs_list(i);
        
        % 位置更新
        obs.x = obs.x + obs.vx * dt;
        obs.y = obs.y + obs.vy * dt;
        
        % (可选) 可以在这里加入变加速逻辑，测试规划器的鲁棒性
        % if i == 1 && obs.x > 60, obs.vx = max(0, obs.vx - 2.0 * dt); end % 示例：前车减速
        
        updated_list(i) = obs;
    end
end