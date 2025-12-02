function [best_idx, best_cost, debug_scores] = evaluate_trajectories(results, obstacles, last_best_target_lane_id, scenario_params, eval_weights)
% EVALUATE_TRAJECTORIES 轨迹评分与择优模块
%
% 逻辑:
%   1. 硬约束过滤: 剔除发散或发生碰撞的轨迹
%   2. 软代价打分: S = J_safety + J_progress + J_comfort + J_consistency
%
% 输入:
%   results: 结构体数组, 包含 .X, .U, .valid, .cand
%   obstacles: 障碍物结构体 (含 .prediction)
%   last_best_target_lane_id: 上一帧选中的目标车道ID (用于一致性奖励)
%   scenario_params: 含 .v_desired (全局期望速度)
%   eval_weights: 权重结构体
%
% 输出:
%   best_idx: 最优轨迹在 results 中的索引
%   best_cost: 最低代价值
%   debug_scores: 调试用，记录每条轨迹的各项分值

    best_cost = inf;
    best_idx = -1;
    num_cands = length(results);
    
    % 初始化调试记录
    debug_scores = struct('id', {}, 'valid', {}, 'J_safe', {}, 'J_prog', {}, 'J_comf', {}, 'J_cons', {}, 'total', {});

    % 警戒距离 (Alert Distance): 大于此距离认为绝对安全，Cost=0
    D_ALERT = 4.0; % 米 (可根据车速动态调整，这里简化为固定值)
    
    % 碰撞容忍度 (Collision Tolerance): 允许微小的数值误差
    D_COLLISION = -0.05; % 稍微留一点负余量防止数值噪声误杀

    for i = 1:num_cands
        traj_X = results(i).X; % 4 x N+1
        traj_U = results(i).U; % 2 x N
        cand = results(i).cand;
        
        % --- 1. 硬约束检查 (Hard Constraints) ---
        
        % A. 算法收敛性检查 (如果 ADMM 没收敛，通常不可信)
        % if ~results(i).success 
        %    record_failure(i, 'Not Converged'); continue; 
        % end
        % (注: 有时即便没完全收敛，轨迹也是可用的，所以这里暂不作硬性过滤，主要靠碰撞检查)

        % B. 碰撞检测 (逐点检查)
        [min_dist, closest_obs_idx] = calc_min_distance_to_obstacles(traj_X, obstacles);
        
        if min_dist < D_COLLISION
            % 发生碰撞，直接淘汰
            record_debug(i, false, inf, 0, 0, 0, inf);
            continue; 
        end

        % --- 2. 软代价计算 (Soft Scoring) ---

        % A. 安全裕度代价 (Quadratic Hinge Loss)
        % J = sum( max(0, d_alert - d)^2 )
        % 只有进入 D_ALERT 范围内的点才产生 Cost
        J_safety = calc_safety_cost(traj_X, obstacles, D_ALERT);
        
        % B. 任务进度代价 (Progress & Efficiency)
        % 奖励 1: 有效行驶距离 (跑得越远越好)
        dist_traveled = traj_X(1, end) - traj_X(1, 1);
        % 惩罚 2: 速度跟踪误差 (相对于全局期望速度)
        v_error_sq = sum((traj_X(4, :) - scenario_params.v_desired).^2);
        
        J_progress = - eval_weights.w_progress * dist_traveled ...
                     + eval_weights.w_ref_vel * v_error_sq;

        % C. 舒适性代价 (Comfort)
        % J = sum( a^2 + jerk^2 )
        acc_sq = sum(traj_U(1, :).^2);
        steer_sq = sum(traj_U(2, :).^2);
        
        % 简单计算 Jerk (差分)
        dt = 0.1; % 假设 dt
        jerk_lon = sum(diff(traj_U(1, :)).^2) / dt^2;
        jerk_lat = sum(diff(traj_U(2, :)).^2) / dt^2;
        
        J_comfort = eval_weights.w_acc * acc_sq + ...
                    eval_weights.w_steer * steer_sq + ...
                    eval_weights.w_jerk_lon * jerk_lon + ...
                    eval_weights.w_jerk_lat * jerk_lat;

        % D. 一致性代价 (Consistency)
        J_consistency = 0;
        if last_best_target_lane_id ~= -1
            if cand.target_lane_id ~= last_best_target_lane_id
                J_consistency = eval_weights.w_consistency;
            end
        end

        % --- 3. 总分汇总 ---
        total_score = eval_weights.w_safety * J_safety + ...
                      J_progress + ...
                      J_comfort + ...
                      J_consistency;

        debug_scores = record_debug(i, true, J_safety, J_progress, J_comfort, J_consistency, total_score, debug_scores);

        % 更新最优
        if total_score < best_cost
            best_cost = total_score;
            best_idx = i;
        end
    end
    
    % --- 内部辅助函数: 记录调试信息 ---
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
% 辅助计算函数
% =========================================================

function J = calc_safety_cost(traj, obstacles, d_alert)
    % 计算二次铰链损失
    J = 0;
    N = size(traj, 2);
    
    for k = 1:N
        ego_pos = traj(1:2, k);
        
        for j = 1:length(obstacles)
            obs = obstacles(j);
            
            % 获取障碍物当前时刻位置 (动态/静态)
            if isfield(obs, 'prediction') && ~isempty(obs.prediction)
                idx = min(k, size(obs.prediction, 2));
                obs_pos = obs.prediction(1:2, idx);
                obs_theta = obs.prediction(3, idx);
            else
                obs_pos = [obs.x; obs.y];
                obs_theta = obs.theta;
            end
            
            % 计算表面距离 (近似)
            dist = point_to_ellipse_dist(ego_pos, obs_pos, obs_theta, obs.a, obs.b);
            
            % Hinge Loss: 只有小于 d_alert 才惩罚
            if dist < d_alert
                J = J + (d_alert - dist)^2;
            end
        end
    end
end

function [min_dist, obs_idx] = calc_min_distance_to_obstacles(traj, obstacles)
    % 寻找整条轨迹中离障碍物的最近距离
    min_dist = inf;
    obs_idx = -1;
    
    N = size(traj, 2);
    for k = 1:N
        ego_pos = traj(1:2, k);
        for j = 1:length(obstacles)
            obs = obstacles(j);
            % 获取位置
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
    % 计算点到椭圆的近似有向距离
    % 正值: 在椭圆外; 负值: 在椭圆内
    
    % 1. 转换到局部坐标系
    dx = pt(1) - center(1);
    dy = pt(2) - center(2);
    
    cos_t = cos(theta);
    sin_t = sin(theta);
    
    local_x =  cos_t * dx + sin_t * dy;
    local_y = -sin_t * dx + cos_t * dy;
    
    % 2. 计算代数距离 metric = (x/a)^2 + (y/b)^2
    metric = (local_x / a)^2 + (local_y / b)^2;
    
    % 3. 转换为近似欧氏距离
    % 这是一个常用的工程近似: dist approx sqrt(metric) * r_effective - r_effective
    % 或者更简单: dist approx (sqrt(metric) - 1) * min(a, b)
    % 为了保守起见，我们使用最小轴缩放，这样算出来的距离比实际偏小（更安全）
    
    dist_approx = (sqrt(metric) - 1.0) * min(a, b);
    
    dist = dist_approx;
end