function [z_u, z_x] = project_constraints(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints)
% PROJECT_CONSTRAINTS_ANTI_DISTORTION
% ADMM 步骤2: 将变量投影到可行域，包含抗扭曲的一致性侧面投影逻辑。
%
% 输入:
%   u_traj:   当前控制轨迹 (2 x N)
%   x_traj:   当前状态轨迹 (4 x N+1)
%   lambda_u: 控制对偶变量
%   lambda_x: 状态对偶变量
%   sigma:    ADMM 惩罚参数
%   constraints: 约束结构体 (.u_min, .u_max, .obstacles)
%
% 输出:
%   z_u: 投影后的控制变量
%   z_x: 投影后的位置变量

% ==========================================
% 1. 控制约束投影 (Box Constraint)
% ==========================================
% 公式: z = clamp(u + lambda/sigma, min, max)
y_u = u_traj + lambda_u / sigma;
z_u = max(constraints.u_min, min(constraints.u_max, y_u));

% ==========================================
% 2. 障碍物约束投影 (Obstacle Avoidance)
% ==========================================
% 提取位置部分 (X, Y)
pos_traj = x_traj(1:2, :);

% 计算待投影点 (ADMM 中间变量)
y_x = pos_traj + lambda_x / sigma;

% 默认输出为原位 (若无碰撞)
z_x = y_x;

% 若无障碍物，直接返回
if ~isfield(constraints, 'obstacles') || isempty(constraints.obstacles)
    return;
end

num_obs = length(constraints.obstacles);
num_steps = size(y_x, 2);

for j = 1:num_obs
    obs = constraints.obstacles(j);

    % --- 2.1 预计算障碍物几何参数 ---
    % 旋转矩阵 R (Global -> Local) 的转置即为 Local -> Global
    cos_t = cos(obs.theta);
    sin_t = sin(obs.theta);
    R = [cos_t, -sin_t; sin_t, cos_t];

    a = obs.a;
    b = obs.b;
    center = [obs.x; obs.y];

    % 快速检测矩阵 A = R * Sigma * R'
    Sigma = diag([1/a^2, 1/b^2]);
    A_mat = R * Sigma * R';

    % --- 2.2 Pass 1: 碰撞检测与全局一致性决策 ---
    % 目的：扫描整条轨迹，决定从障碍物的哪一侧（上方或下方）绕行，防止轨迹扭曲。

    colliding_indices = zeros(1, num_steps);
    colliding_points_local_v = zeros(1, num_steps);
    count = 0; % 计数器

    for k = 1:num_steps
        pt_global = y_x(:, k);
        d_xy = pt_global - center;

        % 椭圆方程检测: d' * A * d < 1 表示在内部
        metric = d_xy' * A_mat * d_xy;

        if metric < 1.0
            count = count + 1;
            colliding_indices(count) = k;

            % 计算局部坐标系的 v (纵坐标)，用于后续判断方向
            pt_local = R' * d_xy;
            colliding_points_local_v(count) = pt_local(2);
        end
    end

    % 截断数组，只保留有效部分
    colliding_indices = colliding_indices(1:count);
    colliding_points_local_v = colliding_points_local_v(1:count);

    % --- 2.3 Pass 2: 执行投影 ---
    if count > 0
        % [决策]: 计算平均纵向偏差。
        % 若 mean_v > 0，说明大部分点在上方，统一向上推；反之向下。
        mean_v = mean(colliding_points_local_v);

        if abs(mean_v) < 1e-2
            target_side_sign = 1.0; % 穿心情况，默认向上
        else
            target_side_sign = sign(mean_v);
        end

        % 对所有碰撞点执行鲁棒投影
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
            % plot(final_x, final_y, 'r-', 'LineWidth', 1, 'DisplayName', 'Obstacle');

            % % 也可以画个中心点方便观察
            % plot(obs.x, obs.y, 'r+', 'HandleVisibility', 'off');
            %% 结束绘图调试用


            % Step A: 转换到局部坐标系 (u, v)
            pt_local = R' * d_xy;
            u = pt_local(1);
            v = pt_local(2);
            % [策略1]: 强制侧面一致性 (Anti-Distortion)
            % 无论当前点 v 是正是负，都强制将其修改为目标符号方向。
            % 这会“欺骗”后续的牛顿法，使其向统一的方向寻找最近点。
            v = target_side_sign * abs(v);
            if abs(v) < 1e-2
                v = target_side_sign * 1e-2;
            end

            % Step B: 利用第一象限对称性简化计算

            % [策略2]: 深度排斥 (Deep Depenetration)
            % 如果点深陷障碍物中心 (norm_dist < 0.3)，常规投影可能不收敛或推力不足。
            % 核心思想: 判断点在归一化坐标下离哪个轴更近，然后沿另一个轴（更远）的方向强力推出。
            % norm_dist = sqrt((u/a)^2 + (v/b)^2);
            % if norm_dist < 0.3
            %     if a < b
            %         u = max(abs(u), 0.1 * a) * sign(u); % 往 x 推
            %     else
            %         v = max(abs(v), 0.1 * b) * sign(v); % 往 y 推
            %     end
            %     % 符号保护
            %     if u == 0, u = 1e-5; end
            %     if v == 0, v = 1e-5; end
            % end
            
            if abs(u) < 0.1 * a
                u = 0.1 * a * sign(u);
            end
            if abs(v) < 0.1 * b
                v = 0.1 * b * sign(v);
            end




            u_abs = abs(u);
            v_abs = abs(v);


            % Step C: 鲁棒牛顿法求解拉格朗日乘子 t
            % 目标方程: f(t) = (a*u / (a^2+t))^2 + (b*v / (b^2+t))^2 - 1 = 0
            t = 0;
            iter_max = 10;
            tol = 1e-4;
            limit_min = -min(a^2, b^2) + 1e-4; % t 的下界 (分母不能为0)
            limit_max = 0;                     % 内部点投影 t 必为负

            for iter = 1:iter_max
                a2_t = a^2 + t;
                b2_t = b^2 + t;

                % 数值稳定性保护
                if abs(a2_t) < 1e-6, a2_t = 1e-6; end
                if abs(b2_t) < 1e-6, b2_t = 1e-6; end

                term_x = (a * u_abs) / a2_t;
                term_y = (b * v_abs) / b2_t;

                f_val = term_x^2 + term_y^2 - 1;

                if abs(f_val) < tol
                    break;
                end

                df_val = -2 * ( (term_x^2)/a2_t + (term_y^2)/b2_t );

                % 导数过小保护
                if abs(df_val) < 1e-10, df_val = -1e-10; end

                % 牛顿迭代步
                t_next = t - f_val / df_val;

                % 越界保护 (Bisection Fallback)
                if t_next <= limit_min
                    t_next = (t + limit_min) / 2;
                elseif t_next >= limit_max
                    t_next = (t + limit_max) / 2;
                end
                t = t_next;
            end

            % Step D: 计算局部投影坐标
            denom_x = a^2 + t;
            denom_y = b^2 + t;

            % 防止除零
            if abs(denom_x) < 1e-8, denom_x = 1e-8; end
            if abs(denom_y) < 1e-8, denom_y = 1e-8; end

            q_x = sign(u) * (a^2 * u_abs) / denom_x;
            % 注意: q_y 的符号跟随之前强制修正过的 v
            q_y = target_side_sign * (b^2 * v_abs) / denom_y;

            % Step E: 转换回全局坐标
            Q_local = [q_x; q_y];
            Q_global = R * Q_local + center;

            % 更新投影结果
            z_x(:, k) = Q_global;


            %% 碰撞投影后绘图调试用
            % plot(Q_global(1), Q_global(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Projected Point');
            % close all;
            %% 结束绘图调试用

            % 更新待处理序列 (防止多障碍物时的顺序依赖问题)
            y_x(:, k) = Q_global;
        end
    end
end
end
