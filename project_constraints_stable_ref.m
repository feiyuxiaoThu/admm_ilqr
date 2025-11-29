function [z_u, z_x, target_dot_sign] = project_constraints_stable_ref(u_traj, x_traj, init_traj, lambda_u, lambda_x, sigma, constraints, target_dot_sign, iter_admm)
% PROJECT_CONSTRAINTS_STABLE_REF (Dynamic Obstacle Support)
% 1. 稳定性核心: 使用 init_traj 确定投影法线方向，避免对称陷阱。
% 2. 动态支持: 在每个时间步 k 读取障碍物的预测位置。
% 3. 投影执行: 射线-椭圆解析求交。

% ==========================================
% 1. 控制约束投影
% ==========================================
y_u = u_traj + lambda_u / sigma;
z_u = max(constraints.u_min, min(constraints.u_max, y_u));

% ==========================================
% 2. 障碍物约束投影
% ==========================================
% ADMM 中间变量 (含噪声的待投影点)
y_x = x_traj(1:2, :) + lambda_x(1:2, :) / sigma;

% 初始化输出 (默认不移动)
z_x = y_x;

if ~isfield(constraints, 'obstacles') || isempty(constraints.obstacles)
    return;
end

num_steps = size(y_x, 2);
num_obs = length(constraints.obstacles);

% --- [Pre-calc]: 基于参考轨迹计算稳定的法向量场 ---
% 这部分只依赖参考线，一次性算好
ref_pos = init_traj(1:2, 1:num_steps);
dx_ref = gradient(ref_pos(1, :));
dy_ref = gradient(ref_pos(2, :));
norm_ref = sqrt(dx_ref.^2 + dy_ref.^2);
norm_ref(norm_ref < 1e-6) = 1;

% 切向量
tx_ref = dx_ref ./ norm_ref;
ty_ref = dy_ref ./ norm_ref;

% 左侧法向量 (Left Normal): [-ty, tx]
nx_ref = -ty_ref;
ny_ref = tx_ref;

% 累积位移场 (所有障碍物的推力叠加)
total_shift = zeros(2, num_steps);

% ==========================================
% 外层循环：遍历障碍物
% ==========================================
for j = 1:num_obs
    obs = constraints.obstacles(j);
    a = obs.a;
    b = obs.b;

    % 检查是否有动态预测轨迹
    % 假设结构: obs.prediction.trajectory (3 x M) -> [x; y; theta]
    is_dynamic = isfield(obs, 'prediction') && ~isempty(obs.prediction);

    % ==========================================
    % 内层循环：遍历时间步 (逐点处理，非矩阵化)
    % ==========================================
    for k = 1:num_steps

        % --- 1. 获取当前时刻 k 的障碍物姿态 ---
        if is_dynamic
            pred_traj = obs.prediction;
            pred_len = size(pred_traj, 2);
            % 防止索引越界 (如果预测不够长，保持最后一帧)
            idx = min(k, pred_len);

            curr_x = pred_traj(1, idx);
            curr_y = pred_traj(2, idx);
            curr_theta = pred_traj(3, idx);
        else
            % 静态障碍物
            curr_x = obs.x;
            curr_y = obs.y;
            curr_theta = obs.theta;
        end

        center = [curr_x; curr_y];

        % 计算当前时刻的旋转矩阵 (Local -> Global)
        cos_t = cos(curr_theta);
        sin_t = sin(curr_theta);
        R = [cos_t, -sin_t; sin_t, cos_t];

        % 计算当前时刻的检测矩阵
        Sigma_inv = diag([1/a^2, 1/b^2]);
        A_mat = R * Sigma_inv * R';

        % --- 2. 碰撞检测 ---
        pt_global = y_x(:, k);
        d_vec = pt_global - center;

        % 椭圆方程检测: d' * A * d
        metric = d_vec' * A_mat * d_vec;

        % 如果没有碰撞，直接跳过当前点
        if metric >= 1.0
            continue;
        end

        % --- 3. [拓扑决策] 判断障碍物在参考线的哪一侧 ---
        % 注意：这里用的是当前时刻 k 的参考点和当前时刻 k 的障碍物中心
        if target_dot_sign(j) == 0
            ref_pt_k = ref_pos(:, k);
            vec_to_center = center - ref_pt_k;

            % 获取当前参考点的切向量
            tx_k = tx_ref(k);
            ty_k = ty_ref(k);

            % 二维叉乘: Ref_Tangent x Vector_To_Obs
            % > 0: 障碍物在左 -> 车往右躲
            % < 0: 障碍物在右 -> 车往左躲
            cross_val = tx_k * vec_to_center(2) - ty_k * vec_to_center(1);

            if cross_val > 0
                target_dot_sign(j) = -1.0; % 目标: 往右 (负法向)
            else
                target_dot_sign(j) = 1.0;  % 目标: 往左 (正法向)
            end
        end


        % --- 4. 准备投影射线 ---
        % 使用参考线的法向量作为投影方向 (稳定性保证)
        n_curr = [nx_ref(k); ny_ref(k)];

        % 奇异值保护
        if norm(n_curr) < 1e-3
            n_curr = [-sin_t; cos_t];
        end

        %% 绘图调试用 (严格保留你的代码结构)
        % figure('Position', [100, 100, 1200, 600]); hold on; axis equal; 
        % plot(pt_global(1), pt_global(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Colliding Point');
        % % 定义单位圆用于变换
        % theta_circle = linspace(0, 2*pi, 100);
        % circle_x = cos(theta_circle);
        % circle_y = sin(theta_circle);
        % % --- 椭圆变换逻辑 ---
        % % 1. 缩放
        % scale_x = obs.a * circle_x;
        % scale_y = obs.b * circle_y;
        % % 2. 旋转 (使用当前时刻的 curr_theta)
        % R_draw = [cos(curr_theta), -sin(curr_theta);
        %     sin(curr_theta),  cos(curr_theta)];
        % rotated_points = R_draw * [scale_x; scale_y];
        % % 3. 平移 (使用当前时刻的 curr_x, curr_y)
        % final_x = rotated_points(1, :) + curr_x;
        % final_y = rotated_points(2, :) + curr_y;
        % % 4. 绘图
        % plot(final_x, final_y, 'r-', 'LineWidth', 1, 'DisplayName', 'Obstacle');
        % % 画中心点
        % plot(curr_x, curr_y, 'r+',  'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
        % plot(x_traj(1, :), x_traj(2, :), 'b-', 'DisplayName', 'Trajectory');
        % plot(y_x(1, :), y_x(2, :), 'g--', 'DisplayName', 'y\_x Points');
        % plot(init_traj(1, :), init_traj(2, :), 'k:', 'DisplayName', 'Reference Traj');
        % xlim([curr_x - obs.a*2, curr_x + obs.a*2]);
        % ylim([curr_y - obs.b*2, curr_y + obs.b*2]);
        %% 结束绘图调试用

        % --- 5. 射线-椭圆 解析求交 ---
        % 射线方程: P(t) = P_local + t * Dir_local

        % 将全局射线转换到障碍物局部坐标系
        % P_local: 碰撞点相对于中心的局部坐标
        pt_local = R' * (pt_global - center);
        u0 = pt_local(1);
        v0 = pt_local(2);

        % Dir_local: 参考线法向量在局部坐标系的投影
        dir_local = R' * n_curr;
        du = dir_local(1);
        dv = dir_local(2);

        % 解一元二次方程: A*t^2 + B*t + C = 0
        % 代入椭圆方程 (u0 + t*du)^2/a^2 + (v0 + t*dv)^2/b^2 = 1
        Coeff_A = (du/a)^2 + (dv/b)^2;
        Coeff_B = 2 * ( (u0*du)/a^2 + (v0*dv)/b^2 );
        Coeff_C = (u0/a)^2 + (v0/b)^2 - 1;

        delta = Coeff_B^2 - 4 * Coeff_A * Coeff_C;

        if delta < 0
            % 理论上若点在椭圆内，必有解。若无解可能是数值误差，跳过
            continue;
        end

        sqrt_delta = sqrt(delta);
        t1 = (-Coeff_B + sqrt_delta) / (2 * Coeff_A);
        t2 = (-Coeff_B - sqrt_delta) / (2 * Coeff_A);

        % 计算两个候选位移向量 (Global Frame)
        % t1, t2 是沿 n_curr 的距离
        shift1 = t1 * n_curr;
        shift2 = t2 * n_curr;

        % --- 6. 目标选择 ---
        % 根据 target_dot_sign 选择正确的 t
        % target > 0 (Left): 选 t 大的 (正向)
        % target < 0 (Right): 选 t 小的 (负向)

        if target_dot_sign(j) > 0
            if t1 > t2, shift_vec = shift1; else, shift_vec = shift2; end
        else
            if t1 < t2, shift_vec = shift1; else, shift_vec = shift2; end
        end

        % 累加推力 (Shift)
        total_shift(:, k) = total_shift(:, k) + shift_vec;

        %% 碰撞投影后绘图调试用
        % Q_global = y_x(:, k) + shift_vec;
        % plot(Q_global(1), Q_global(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Projected Point');
        % legend('Location', 'best');
        % title(sprintf('Obstacle %d at Time Step %d, ADMM iter %d', j, k, iter_admm));
        % close all;
        %% 结束绘图调试用

    end % end k loop
end % end obs loop

% 3. 应用合力并执行道路边界截断 (Strategy B)

% 先应用避障推力
z_x_temp = y_x + total_shift;

% 再执行硬截断
if isfield(constraints, 'road_bounds') && ~isempty(constraints.road_bounds)
    y_min = constraints.road_bounds(1);
    y_max = constraints.road_bounds(2);
    
    % 对 Y 轴 (第2行) 进行 Clamp
    % 逻辑: max(y_min, min(y_max, val))
    z_x_temp(2, :) = max(y_min, min(y_max, z_x_temp(2, :)));
end

z_x = z_x_temp;

end
