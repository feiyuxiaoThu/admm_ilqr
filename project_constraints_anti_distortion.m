function [z_u, z_x] = project_constraints_anti_distortion(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints)
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
        % ... (A_mat, R, a, b, center 等参数预计算保持不变)
        cos_t = cos(obs.theta);
        sin_t = sin(obs.theta);
        R = [cos_t, -sin_t; sin_t, cos_t];
        a = obs.a; b = obs.b; center = [obs.x; obs.y];
        Sigma = diag([1/a^2, 1/b^2]);
        A_mat = R * Sigma * R';
        %%% --- START: Consistent Side Projection Logic --- %%%

        % 1. 识别所有与该障碍物碰撞的点
        colliding_indices = zeros(1, num_steps);
        colliding_points_local_v = zeros(1, num_steps,1); % 存储碰撞点在局部坐标系的 v 值
        colliding_index = 0;
        for k = 1:num_steps
            pt_global = y_x(:, k);
            d_xy = pt_global - center;
            metric = d_xy' * A_mat * d_xy;

            if metric < 1.0
                colliding_index = colliding_index + 1;
                colliding_indices(1, colliding_index) = k;
                pt_local = R' * d_xy;
                colliding_points_local_v(1, colliding_index) = pt_local(2);
            end
        end
        colliding_indices = colliding_indices(1, 1:colliding_index);
        colliding_points_local_v = colliding_points_local_v(1, 1:colliding_index);

        % 2. 如果有碰撞，进行集体决策
        if ~isempty(colliding_indices)
            % 决策：根据碰撞点的 v 坐标均值，决定是从上方(v>0)还是下方(v<0)绕
            % 这可以避免轨迹在障碍物两侧震荡
            mean_v = mean(colliding_points_local_v);

            % 如果均值接近0 (穿心)，默认选择 +v 方向
            if abs(mean_v) < 1e-2
                target_side_sign = 1.0;
            else
                target_side_sign = sign(mean_v);
            end
            % 3. 对所有碰撞点执行“有偏向”的投影
            for k = colliding_indices
                pt_global = y_x(:, k);
                d_xy = pt_global - center;

                %% 绘图调试用
                % figure; hold on; axis equal;
                % plot(pt_global(1), pt_global(2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Colliding Point');
                % % 定义单位圆用于变换
                % theta_circle = linspace(0, 2*pi, 100);
                % circle_x = cos(theta_circle);
                % circle_y = sin(theta_circle);
                % % --- 椭圆变换逻辑 ---
                % % 1. 缩放 (根据半长轴 a 和 半短轴 b)
                % scale_x = obs.a * circle_x;
                % scale_y = obs.b * circle_y;
                
                % % 2. 旋转 (根据 heading theta)
                % R = [cos(obs.theta), -sin(obs.theta);
                %     sin(obs.theta),  cos(obs.theta)];
                % rotated_points = R * [scale_x; scale_y];
                
                % % 3. 平移 (移动到中心 x, y)
                % final_x = rotated_points(1, :) + obs.x;
                % final_y = rotated_points(2, :) + obs.y;
                
                % % 4. 绘图 (使用 fill 填充颜色)
                % fill(final_x, final_y, [1, 0.6, 0.6], ...
                %     'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 1, ...
                %     'DisplayName', 'Obstacle');
                
                % % 也可以画个中心点方便观察
                % plot(obs.x, obs.y, 'r+', 'HandleVisibility', 'off');
                %% 结束绘图调试用

                % Step A: 转局部坐标
                pt_local = R' * d_xy;
                u = pt_local(1);
                v = pt_local(2);

                % 强制所有点都在决策好的那一侧进行投影,通过修改输入给牛顿法的 v 值来实现这一点
                % 这会“欺骗”投影算法，让它认为点在期望的一侧
                v = target_side_sign * abs(v);
                if abs(v) < 1e-2 % 如果刚好在长轴上，给个小扰动
                    v = target_side_sign * 1e-2;
                end

                % Step B: 利用第一象限对称性 (u_abs, v_abs)
                u_abs = abs(u);
                v_abs = abs(v);

                % Symmetry Breaking，防止初始轨迹刚好穿过障碍物椭圆中心时陷入局部最优
                norm_dist = sqrt((u/a)^2 + (v/b)^2);
                % 如果点深陷在障碍物内部 (例如核心 30% 区域)
                if norm_dist < 0.3
                    % 既然在中心附近，说明之前的迭代没能把点推出去。
                    % 我们人工指定：往“容易出去”的方向(短轴)推！
                    if a < b
                        % x轴更短，往 x 轴推
                        u_abs = max(u_abs, 0.1 * a);
                        % 这里的 0.1*a 是为了给牛顿法一个显著的非零初值
                    else
                        % y轴更短，往 y 轴推
                        v_abs = max(v_abs, 0.1 * b);
                    end
                    
                    % 如果是0，就随便给个正方向
                    if u == 0, u = 1e-5; end
                    if v == 0, v = 1e-5; end
                end

                % Step C: 牛顿法求解 t (代码不变)
                % ... (将你之前优化的鲁棒牛顿法代码粘贴在这里) ...
                t = 0;
                iter_max = 10; tol = 1e-4;
                limit_min = -min(a^2, b^2) + 1e-4;
                limit_max = 0;
                for iter = 1:iter_max
                    a2_t = a^2 + t; 
                    b2_t = b^2 + t;
                    % 防止分母为0 (极少数情况，如点在焦点上，加个epsilon保护)
                    if abs(a2_t) < 1e-6
                         a2_t = 1e-6; 
                    end
                    if abs(b2_t) < 1e-6
                         b2_t = 1e-6;
                    end

                    % 计算 f(t)
                    term_x = (a * u_abs) / a2_t;
                    term_y = (b * v_abs) / b2_t;
                    f_val = term_x^2 + term_y^2 - 1;

                    % 检查收敛
                    if abs(f_val) < tol
                        break;
                    end

                    % 计算 f'(t)
                    % f'(t) = -2 * [ (a^2*u^2)/(a^2+t)^3 + (b^2*v^2)/(b^2+t)^3 ]
                    df_val = -2 * ( (term_x^2)/a2_t + (term_y^2)/b2_t );

                    %导数过小保护 (防止除以0产生NaN)
                    if abs(df_val) < 1e-10
                         % 如果导数消失（通常不会发生，除非在中心），退化为梯度下降或直接跳出
                         df_val = -1e-10; 
                    end
                    % 牛顿步
                    t_next = t - f_val / df_val;
                    if t_next <= limit_min
                        t_next = (t + limit_min) / 2;
                    elseif t_next >= limit_max
                        t_next = (t + limit_max) / 2;
                    end
                    t = t_next;
                end
                % Step D & E: 计算投影点并转回全局坐标 (代码微调)
                denom_x = a^2 + t; denom_y = b^2 + t;
                if abs(denom_x) < 1e-8, denom_x = 1e-8; end
                if abs(denom_y) < 1e-8, denom_y = 1e-8; end
                q_x = sign(u) * (a^2 * u_abs) / denom_x;
                % v的符号在前面已经被我们强制修正了
                q_y = v * (b^2) / denom_y;

                Q_local = [q_x; q_y];
                Q_global = R * Q_local + center;

                % 更新 z_x 轨迹
                z_x(:, k) = Q_global;
                
                %% 碰撞投影后绘图调试用
                % plot(Q_global(1), Q_global(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Projected Point');
                % close all;
                %% 结束绘图调试用

                % 更新 y_x (可选，防止多重障碍物处理时的顺序问题)
                y_x(:, k) = z_x(:, k);
            end
        end
        %%% --- END: Consistent Side Projection Logic --- %%%
    end
end
end
