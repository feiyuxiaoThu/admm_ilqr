function candidates = generate_candidates_structured(x0, scenario, constraints, N, dt)
% GENERATE_CANDIDATES_STRUCTURED
% 基于 target_velocity/position_solver 生成多模态候选轨迹
%
% 输入:
%   x0: 自车状态 [x; y; theta; v; a; kappa] (注意: 这里假设输入包含加速度a)
%       如果 x0 只有 4 维，则 a 默认为 0
%   scenario: 包含 lane_centers, v_desired
%   constraints: 包含 obstacles, u_max, u_min, limits (jerk limits)
%   N, dt: 规划步数和步长

% --- 1. 解析输入 ---
p0_x = x0(1);
p0_y = x0(2);
theta0 = x0(3);
v0 = x0(4);
if length(x0) >= 5, a0 = x0(5); else, a0 = 0; end

% 物理限制 (用于 Solver)
% 默认值 (参考论文)
limits.a_max = constraints.u_max(1);
limits.a_min = constraints.u_min(1);
limits.j_max = 1.0; % m/s^3
limits.j_min = -1.0;

v_max = 20.0; % 车辆物理极限或道路限速
v_min = 0.0;
v_desired = scenario.v_desired;

candidates = [];
cand_id = 1;

% --- 2. 遍历横向意图 (Lanes) ---
% 查找当前车道索引
[~, curr_lane_idx] = min(abs(scenario.lane_centers - p0_y));

% 定义搜索方向: [当前, 左, 右] (索引偏移)
search_offsets = [0, 1, -1];

for offset = search_offsets
    target_lane_idx = curr_lane_idx + offset;
    
    % 检查车道是否存在
    if target_lane_idx < 1 || target_lane_idx > length(scenario.lane_centers)
        continue;
    end
    
    target_y = scenario.lane_centers(target_lane_idx);
    
    % 定义横向动作标签
    if offset == 0, lat_action = 'Keep';
    elseif offset == 1, lat_action = 'Left';
    else, lat_action = 'Right';
    end
    
    % --- 3. 感知: 寻找该车道的前方障碍物 ---
    [has_lead, lead_obs] = find_leading_obstacle(p0_x, target_y, constraints.obstacles);
    
    % ===========================================================
    % 纵向模式 A: 达到期望车速 (Cruise / Overtake)
    % ===========================================================
    % 只要不是静止障碍物堵死，就可以尝试生成这个意图
    % (即使有慢车，生成这个意图也代表"尝试超越"或"逼近"，由后续评估层决定是否安全)
    
    % 调用速度求解器
    P_cruise = target_velocity_solver(v0, a0, v_desired, limits);
    % plot_target_velocity_result(P_cruise, v0, a0, p0_x, limits, N*dt);
    
    % 生成参考轨迹点
    [traj_cruise, acc_cruise] = discretize_trajectory(P_cruise, 'velocity', x0, target_y, N, dt);
    % figure;
    % subplot(2,1,1);
    % plot(traj_cruise(1, :), traj_cruise(2, :), 'LineWidth', 1.0);
    % legend('Cruise Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on;
    % subplot(2,1,2);
    % plot(0:dt:dt*N, traj_cruise(4, :), 'LineWidth', 1.0);
    % legend('Velocity Profile'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;
    % close all;
    
    % 添加候选
    candidates = add_candidate(candidates, cand_id, ...
        [lat_action, '_Cruise'], traj_cruise, acc_cruise, v_desired, target_lane_idx);
    cand_id = cand_id + 1;
    
    % ===========================================================
    % 纵向模式 B: 适应前车 (Follow / Yield / Stop)
    % ===========================================================
    if has_lead
        % 获取前车信息
        v_lead = lead_obs.vx; % 假设只有纵向速度
        p_lead = lead_obs.x;
        safe_dist = 10.0 + v_lead * 1.5; % 简单的安全距离模型: 固定 + 时距
        
        if v_lead > 1.0
            % --- B1: 前车在动 -> 速度匹配 (Target Velocity) ---
            % 目标: 前车速度和目标车速取小值
            target_v = min(v_lead, v_desired);
            P_follow = target_velocity_solver(v0, a0, target_v, limits);
            % plot_target_velocity_result(P_follow, v0, a0, p0_x, limits, N*dt);
            % close all;
            
            % 生成轨迹
            [traj_follow, acc_follow] = discretize_trajectory(P_follow, 'velocity', x0, target_y, N, dt);
            % figure;
            % subplot(2,1,1);
            % plot(traj_follow(1, :), traj_follow(2, :), 'LineWidth', 1.0);
            % legend('Cruise Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); axis equal; grid on;
            % subplot(2,1,2);
            % plot(0:dt:dt*N, traj_follow(4, :), 'LineWidth', 1.0);
            % legend('Velocity Profile'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;
            % close all;
            
            % 修正: 速度求解器不保证位置。生成的轨迹可能会撞上。
            % 在这里我们不做截断，保留这个"速度匹配"的意图。
            % 如果最终位置 p > p_lead - safe_dist，后续 ADMM 的 safe cost 会惩罚它。
            
            candidates = add_candidate(candidates, cand_id, ...
                [lat_action, '_Follow_V'], traj_follow, acc_follow, target_v, target_lane_idx);
            cand_id = cand_id + 1;
            
        else
            % --- B2: 前车静止 -> 位置停车 (Target Position) ---
            target_pos = p_lead - safe_dist;
            
            if target_pos > p0_x
                % limits.a_max = 0.0; % 只能减速
                [Pa, Pb, tpb, tc, t_total] = target_position_solver(p0_x, v0, a0, target_pos, limits, v_max, v_min);

                % target_position_results.Pa = Pa;
                % target_position_results.Pb = Pb;
                % target_position_results.tpb = tpb;
                % target_position_results.tc = tc;
                % target_position_results.T_total = t_total;
                % plot_target_position_result(target_position_results, p0_x, v0, a0, limits, dt, 15, target_pos);
                % close all;
                
                % 封装位置求解器的输出信息
                solver_out.Pa = Pa;
                solver_out.Pb = Pb;
                solver_out.tpb = tpb;
                solver_out.tc = tc;
                
                % 生成轨迹
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
end

% =========================================================
% 辅助函数
% =========================================================

function [found, obs] = find_leading_obstacle(ego_x, lane_y, obstacles)
% 寻找指定车道前方最近的障碍物
min_dist = inf;
found = false;
obs = [];
lane_threshold = 1.5; % 车道宽的一半左右，判断障碍物是否在该车道

for i = 1:length(obstacles)
    o = obstacles(i);
    % 检查是否在前方
    if o.x > ego_x
        % 检查横向距离
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
c.x_ref = traj; % [x; y; theta; v] (4xN)
c.v_target = v_tgt; % 用于 Initial Guess
c.acc = acc; % 纵向加速度
c.target_lane_id = lane_id;

if isempty(list)
    new_list = c;
else
    new_list = [list, c];
end
end

function [traj, acc] = discretize_trajectory(P_struct, mode, x0, target_y, N, dt)
% P_struct: 可能是 velocity solver 的 P，也可能是 position solver 的 S

traj = zeros(4, N+1); % [x, y, theta, v]
acc = zeros(1, N+1);

p0_x = x0(1);
p0_y = x0(2);
theta0 = x0(3); % 初始航向
v0 = x0(4);
if length(x0)>=5, a0=x0(5); else, a0=0; end

% 假设换道耗时 4.0 秒
T_lane_change = 4.0;

for k = 1:N+1
    t = (k-1) * dt;
    
    % --- 1. 纵向计算 (调用 get_state_at_t) ---
    if strcmp(mode, 'velocity')
        % 直接调用现成的 get_state_at_t
        % P_struct 就是 P
        [p_val, v_val, a_val] = get_state_at_t(v0, a0, p0_x, P_struct, t);
        
    else % mode == 'position'
        % 需要一个 Wrapper 来处理 Pa -> Cruise -> Pb 的拼接
        % P_struct 就是 position solver 输出的结构体 (含 Pa, Pb, tpb, tc)
        [p_val, v_val, a_val] = evaluate_position(P_struct, v0, a0, p0_x, t);
    end
    
    % --- 2. 横向计算 (平滑插值) ---
    if t < T_lane_change
        ratio = t / T_lane_change;
        s = 10*ratio^3 - 15*ratio^4 + 6*ratio^5; % 五次多项式
        y_val = p0_y + (target_y - p0_y) * s;
    else
        y_val = target_y;
    end
    
    % --- 3. 航向角近似 ---
    if k > 1
        dx = p_val - traj(1, k-1);
        dy = y_val - traj(2, k-1);
        % 防止原地不动时角度乱跳
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