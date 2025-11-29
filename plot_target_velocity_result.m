function plot_target_velocity_result(P, v0, a0, p0, limits, total_time, dt)
% PLOT_TARGET_VELOCITY_RESULT 可视化速度规划求解器的结果
%
% 输入:
%   P: target_velocity_solver 的输出结构体
%   v0, a0, p0: 初始状态 (p0 默认为 0)
%   limits: 用于绘图边界 (可选)
%   total_time: 绘图总时长 (如果为空，默认为 P.T3 + 1s)
%   dt: 绘图采样步长 (默认 0.01s)

    if nargin < 4, p0 = 0; end
    if nargin < 6 || isempty(total_time), total_time = P.T3 * 1.2; end % 多画 20%
    if nargin < 7, dt = 0.01; end

    time_vec = 0:dt:total_time;
    N = length(time_vec);
    
    p_hist = zeros(1, N);
    v_hist = zeros(1, N);
    a_hist = zeros(1, N);
    j_hist = zeros(1, N);
    
    % 采样计算
    for k = 1:N
        t = time_vec(k);
        % 获取状态
        [p, v, a] = get_state_at_t(v0, a0, p0, P, t);
        
        p_hist(k) = p;
        v_hist(k) = v;
        a_hist(k) = a;
        
        % 获取 jerk (分段常数)
        if t <= P.T1
            j_hist(k) = P.j1;
        elseif t <= P.T2
            j_hist(k) = P.j2; % usually 0
        elseif t <= P.T3
            j_hist(k) = P.j3;
        else
            j_hist(k) = 0;
        end
    end
    
    % 绘图
    figure('Name', 'Target Velocity Solver Result', 'Color', 'w');
    sgtitle('Target Velocity Profile (Double S)');
    
    % 1. Position
    subplot(4, 1, 1);
    plot(time_vec, p_hist, 'LineWidth', 1); grid on;
    ylabel('Pos (m)');
    xline(P.T3, 'r--', 'T_{end}');
    
    % 2. Velocity
    subplot(4, 1, 2);
    plot(time_vec, v_hist, 'LineWidth', 1); grid on;
    ylabel('Vel (m/s)');
    yline(v0, 'g--', 'v_{start}');
    yline(v_hist(end), 'b--', 'v_{end}');
    
    % 3. Acceleration
    subplot(4, 1, 3);
    plot(time_vec, a_hist, 'LineWidth', 1); grid on;
    ylabel('Acc (m/s^2)');
    if nargin >= 5
        yline(limits.a_max, 'k:'); yline(limits.a_min, 'k:');
    end
    
    % 4. Jerk
    subplot(4, 1, 4);
    stairs(time_vec, j_hist, 'LineWidth', 1); grid on; % stairs for step function
    ylabel('Jerk (m/s^3)');
    xlabel('Time (s)');
    if nargin >= 5
        yline(limits.j_max, 'k:'); yline(limits.j_min, 'k:');
    end
end