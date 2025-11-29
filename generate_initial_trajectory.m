function [X, U] = generate_initial_trajectory(x0, x_ref_traj, N, dt, L, constraints, acc_ref)
% GENERATE_INITIAL_TRAJECTORY
% 使用 Pure Pursuit (横向) 和 P-Control (纵向) 生成热启动轨迹
%
% 输入:
%   x0: 初始状态 [X; Y; phi; v]
%   x_ref_traj: 参考轨迹 (4 x N+1)
%   N: 时间步数
%   dt: 时间间隔
%   L: 轴距
%   constraints: 约束结构体 (.u_min, .u_max)
% 输出:
%   X: 状态轨迹 (4 x N+1)
%   U: 控制轨迹 (2 x N)

nx = 4; nu = 2;
U = zeros(nu, N);
X = zeros(nx, N+1);

X(:, 1) = x0;

% --- 控制器参数 ---
Kp_vel = 3.0;       % 速度 P 增益 (越大约激进)
Ld_min = 5.0;       % 最小预瞄距离 (m)
kv = 1.0;           % 预瞄距离随速度增益 (Ld = Ld_min + kv * v)

if nargin > 6 && ~isempty(acc_ref) && length(acc_ref) >= N+1
    % 如果提供了参考加速度，则使用它来辅助纵向控制
    use_acc_ref = true;
else
    use_acc_ref = false;
end

% 遍历每个时间步生成控制量
for k = 1:N
    % 当前状态
    curr_x = X(1, k);
    curr_y = X(2, k);
    curr_phi = X(3, k);
    curr_v = X(4, k);

    % --- 1. 纵向控制 (P Control) ---
    % 获取当前时刻对应的参考速度
    % 注意：x_ref_traj 可能比 k 长，防止越界
    ref_idx = min(k, size(x_ref_traj, 2));
    ref_v = x_ref_traj(4, ref_idx);

    % 计算加速度 (简单 P 控制)
    if use_acc_ref
        des_a = acc_ref(ref_idx);
    else
        des_a = Kp_vel * (ref_v - curr_v);
        if (abs(ref_v) < 0.1) && (curr_v < 0.1)
            des_a = -curr_v / dt; % 快速刹停
        end
    end


    % --- 2. 横向控制 (Pure Pursuit) ---
    % 计算动态预瞄距离
    Ld = Ld_min + kv * curr_v;

    % 在参考轨迹上寻找目标点
    % 策略：从当前参考点 ref_idx 开始往后找，找到第一个距离大于 Ld 的点
    target_idx = ref_idx;
    found_target = false;

    for j = ref_idx : size(x_ref_traj, 2)
        dx = x_ref_traj(1, j) - curr_x;
        dy = x_ref_traj(2, j) - curr_y;
        dist = sqrt(dx^2 + dy^2);

        if dist >= Ld
            target_idx = j;
            found_target = true;
            break;
        end
    end

    % 如果跑到终点还没找到足够远的点，就取最后一个点
    if ~found_target
        target_idx = size(x_ref_traj, 2);
    end

    target_x = x_ref_traj(1, target_idx);
    target_y = x_ref_traj(2, target_idx);

    % 转换到车辆局部坐标系
    dx = target_x - curr_x;
    dy = target_y - curr_y;

    local_y = -sin(curr_phi) * dx + cos(curr_phi) * dy;

    % 实际距离 (重新计算，因为目标点可能变了)
    Ld_actual = sqrt(dx^2 + dy^2);

    % Pure Pursuit 公式: delta = atan(2 * L * sin(alpha) / Ld)
    % sin(alpha) = local_y / Ld_actual
    % => delta = atan(2 * L * local_y / (Ld_actual^2))

    if Ld_actual < 1e-3 % 防止除零 (虽然很难发生)
        des_delta = 0;
    else
        des_delta = atan((2 * L * local_y) / (Ld_actual^2));
    end

    % --- 3. 约束截断 ---
    % 必须确保初始轨迹满足约束，否则ADMM一开始就会面临不可行的对偶变量
    a_clamped = max(constraints.u_min(1), min(constraints.u_max(1), des_a));
    delta_clamped = max(constraints.u_min(2), min(constraints.u_max(2), des_delta));

    U(:, k) = [a_clamped; delta_clamped];

    % --- 4. 状态更新 ---
    X(:, k+1) = update_state(X(:, k), U(:, k), dt, L);
end
end
