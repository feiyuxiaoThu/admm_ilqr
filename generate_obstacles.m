function obstacles = generate_obstacles(raw_obs_list, ego_size, dt, N)
% GENERATE_OBSTACLES 
% 1. 将长方形障碍物膨胀为安全椭圆 (考虑自车尺寸)
% 2. 基于恒定速度模型 (CV) 预测障碍物在未来 N 步的轨迹
%
% Inputs:
%   raw_obs_list: struct array, 包含 x, y, length, width, theta, vx, vy
%   ego_size: struct, 包含 length, width
%   dt, N: 预测步长和步数
%
% Output:
%   obstacles: 处理后的结构体，包含 a, b (半轴) 和 prediction (2xN 轨迹)

obstacles = [];
if isempty(raw_obs_list)
    return;
end

for i = 1:length(raw_obs_list)
    raw = raw_obs_list(i);
    obs = raw; % 复制基础属性
    
    % 1. 几何膨胀 (Minkowski Sum 近似)
    % 椭圆半长轴 a = (障碍长 + 自车长) / 2 + 安全余量
    % 椭圆半短轴 b = (障碍宽 + 自车宽) / 2 + 安全余量
    safe_margin_lon = 3.0; % 额外的安全距离 (m)
    safe_margin_lat = 0.8;
    obs.a = (raw.length + ego_size.length) / 2.0 + safe_margin_lon;
    obs.b = (raw.width  + ego_size.width)  / 2.0 + safe_margin_lat;
    
    % 2. 轨迹预测 (Constant Velocity Model)
    % 生成 2 x N 的矩阵，表示未来 t = 0 到 t = (N-1)*dt 的位置
    % prediction(:, k) 对应第 k 步的位置

    if (norm([raw.vx; raw.vy]) < 1e-3)
        % 静止障碍物
       obs.prediction = [];
       obstacles = [obstacles, obs];
       continue;
    end
    
    % 时间向量 1xN: [0, dt, 2dt, ..., (N-1)dt]
    t_seq = (0:N-1) * dt;
    
    % 简单的线性运动学方程: p(t) = p0 + v * t
    pred_x = raw.x + raw.vx * t_seq;
    pred_y = raw.y + raw.vy * t_seq;
    pred_theta = repmat(raw.theta, 1, N); % 假设航向不变
    
    obs.prediction = [pred_x; pred_y; pred_theta];
    
    obstacles = [obstacles, obs];
end
end
