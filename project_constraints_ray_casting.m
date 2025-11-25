function [z_u, z_x] = project_constraints_ray_casting(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints)
    % PROJECT_CONSTRAINTS ADMM步骤2: 将变量投影到可行域
    %
    % 输入:
    %   u_traj: 当前控制轨迹 (2 x N)
    %   x_traj: 当前状态轨迹 (4 x N+1)
    %   lambda_u: 控制对偶变量 (2 x N)
    %   lambda_x: 状态对偶变量 (2 x N+1) -- 注意只取前两维(X,Y)
    %   sigma: 惩罚参数
    %   constraints: 约束参数结构体
    %       .u_min, .u_max: 控制量范围
    %       .obstacles: 障碍物列表 struct array (.x, .y, .a, .b, .theta)
    %
    % 输出:
    %   z_u: 投影后的控制变量 (2 x N)
    %   z_x: 投影后的位置变量 (2 x N+1)
    
    % ==========================================
    % 1. 控制约束投影 (Box Constraint)
    % ==========================================
    % 计算待投影点 y_u = u + lambda_u / sigma
    y_u = u_traj + lambda_u / sigma;
    
    % 执行截断 (Clamp)
    z_u = max(constraints.u_min, min(constraints.u_max, y_u));
    
    % ==========================================
    % 2. 障碍物约束投影 (Obstacle Avoidance)
    % ==========================================
    % 提取位置部分 (X, Y)
    pos_traj = x_traj(1:2, :); 
    
    % 计算待投影点 y_x = x + lambda_x / sigma
    % 注意: lambda_x 也是 2x(N+1) 维，只对应位置
    y_x = pos_traj + lambda_x / sigma;
    
    z_x = y_x; % 默认就在原位 (如果没有碰撞)
    
    if isfield(constraints, 'obstacles') && ~isempty(constraints.obstacles)
        num_obs = length(constraints.obstacles);
        num_steps = size(y_x, 2);
        
        for j = 1:num_obs
            obs = constraints.obstacles(j);
            
            % === 1. 构建论文公式 (9) 的 A 矩阵 ===
            % 注意：论文里的 e_a, e_b 对应半轴长
            cos_t = cos(obs.theta);
            sin_t = sin(obs.theta);
            R = [cos_t, -sin_t; sin_t, cos_t]; % 公式 (7)
            
            Sigma = diag([1/obs.a^2, 1/obs.b^2]);
            A_mat = R * Sigma * R'; % 公式 (9)
            
            % 障碍物中心
            center = [obs.x; obs.y];
            
            for k = 1:num_steps
                pt = y_x(:, k);
                d_xy = pt - center; % 向量差
                
                % === 2. 检查约束公式 (8) ===
                % h(x) = 1 - d_xy' * A * d_xy <= 0  ==>  d_xy' * A * d_xy >= 1
                metric = d_xy' * A_mat * d_xy;
                
                if metric < 1.0
                    % === 碰撞 (在椭圆内) ===
                    % 需要找到边界上的投影点 z。
                    % 使得 z 满足 (z-c)'*A*(z-c) = 1，且 ||z - pt|| 最小。
                    
                    % 使用简化的几何投影 (依然是最稳健的方法)
                    % 虽然判别用了矩阵A，但在求投影点时，转回局部坐标系求射线交点
                    % 依然是求解 metric = 1 的最快数值方法。
                    
                    % 这里的 metric 其实就是 (r_local / r_boundary)^2
                    % 所以 scaling factor = 1 / sqrt(metric)
                    
                    % 这种简单的缩放对应于“中心射线投影”
                    % z = center + d_xy / sqrt(metric)
                    
                    z_x(:, k) = center + d_xy / sqrt(metric);
                    
                    % 更新 y_x 以应对重叠障碍物 (可选)
                    y_x(:, k) = z_x(:, k); 
                end
            end
        end
    end
end
