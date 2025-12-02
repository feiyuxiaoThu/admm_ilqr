function [scenario, constraints, x0, raw_obs_list] = build_simulation_scenario(dt, N)
% BUILD_SIMULATION_SCENARIO 构建三车道动静态障碍物场景
% (增加了道路边界定义)

    % --- 1. 道路几何 (三车道, 宽4m) ---
    % Lane 1 (Right): y = -4
    % Lane 2 (Center): y = 0
    % Lane 3 (Left): y = 4
    scenario.lane_centers = [-4.0, 0.0, 4.0];
    scenario.lane_width = 4.0;
    
    % --- 2. 自车初始状态 ---
    % 位于中车道，速度 10/s
    x0 = [0.0; 0.0; 0.0; 10.0];
    
    % 自车尺寸
    ego_size.length = 4.5;
    ego_size.width = 2.0;
    scenario.ego_size = ego_size;
    
    % --- 3. [新增] 计算道路边界约束 (针对质心) ---
    % 物理边缘: 最左车道中心 + 半宽，最右车道中心 - 半宽
    road_edge_left  = max(scenario.lane_centers) + scenario.lane_width/2; % +6.0
    road_edge_right = min(scenario.lane_centers) - scenario.lane_width/2; % -6.0
    
    % 质心允许范围: 物理边缘向内收缩半个车宽 + 安全余量
    margin_safety = 0.2; % 0.2m 的额外缓冲
    y_max = road_edge_left  - ego_size.width/2 - margin_safety;
    y_min = road_edge_right + ego_size.width/2 + margin_safety;
    
    constraints.road_bounds = [y_min, y_max]; % [-4.8, 4.8]
    
    % --- 4. 障碍物生成 ---
    % Obs 1: 中车道前方慢车
    obs1 = struct('x', 40.0, 'y', 0.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 6.0, 'vy', 0.0);
    % Obs 2: 左车道并行
    obs2 = struct('x', 5.0, 'y', 4.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 8.0, 'vy', 0.0);
    % Obs 3: 右车道前方
    obs3 = struct('x', 60.0, 'y', -4.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 0.0, 'vy', 0.0);
    
    raw_obs_list = [obs1, obs2, obs3];
    constraints.obstacles = generate_obstacles(raw_obs_list, ego_size, dt, N);
    
    % --- 5. 物理约束 ---
    constraints.u_min = [-2.0; -0.5]; 
    constraints.u_max = [ 2.0;  0.5]; 
    
    % 场景元数据
    scenario.v_desired = 20.0; 
end