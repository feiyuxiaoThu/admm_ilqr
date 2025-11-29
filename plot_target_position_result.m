function plot_target_position_result(S, p0, v0, a0, limits, dt, plot_time, target_pos)
% PLOT_TARGET_POSITION_RESULT 可视化位置规划求解器的结果
%
% 输入:
%   S: 包含 target_position_solver 输出的结构体 (S.Pa, S.Pb, S.tpb, S.tc)
%      或者直接传入 Pa, Pb, tpb, tc
%   p0, v0, a0: 初始状态
%   limits: 约束 (可选)
%   dt: 采样步长

% 解析输入 (支持结构体或直接参数，这里假设用结构体方便封装)
% 假设调用前已经把 [Pa, Pb, tpb, tc] 打包成 S
Pa = S.Pa;
Pb = S.Pb;
tpb = S.tpb;
tc = S.tc;

time_vec = 0:dt:plot_time;
N = length(time_vec);

p_hist = zeros(1, N);
v_hist = zeros(1, N);
a_hist = zeros(1, N);
j_hist = zeros(1, N);

% 预计算中间状态点，用于分段调用
% 1. Pa 阶段结束 (tpb)
[ppb, vpb, apb] = get_state_at_t(v0, a0, p0, Pa, tpb);

% 2. Cruise 阶段结束
p_cruise_end = ppb + vpb * tc;

for k = 1:N
    t = time_vec(k);

    if t <= tpb
        % Phase A: Acceleration / Adjustment
        [p, v, a] = get_state_at_t(v0, a0, p0, Pa, t);

        % Jerk lookup for Pa
        if t <= Pa.T1, j = Pa.j1; %#ok<ALIGN>
        elseif t <= Pa.T2, j = Pa.j2;
        elseif t <= Pa.T3, j = Pa.j3;
        else, j = 0; end

    elseif t <= (tpb + tc)
        % Phase C: Constant Velocity Cruise
        dt_cruise = t - tpb;
        p = ppb + vpb * dt_cruise;
        v = vpb;
        a = 0;
        j = 0;
    else
        if tc > 0
            a_start_b = 0; % If there was cruise, start decel from 0 acc
        else
            a_start_b = apb; % Otherwise inherit acc from Pa
        end
        t_in_b = t - (tpb + tc);

        if t_in_b > Pb.T3
            [p_final, v_final, a_final] = get_state_at_t(vpb, a_start_b, p_cruise_end, Pb, Pb.T3);
            p = p_final;
            v = v_final;
            a = a_final;
            j = 0;
        else
            [p, v, a] = get_state_at_t(vpb, a_start_b, p_cruise_end, Pb, t_in_b);
            if t_in_b <= Pb.T1, j = Pb.j1; %#ok<ALIGN>
            elseif t_in_b <= Pb.T2, j = Pb.j2; 
            elseif t_in_b <= Pb.T3, j = Pb.j3;
            else, j = 0; end
        end
    end


    p_hist(k) = p;
    v_hist(k) = v;
    a_hist(k) = a;
    j_hist(k) = j;
end

% 绘图
figure('Name', 'Target Position Solver Result', 'Color', 'w');
sgtitle('Target Position Profile (Pa -> Cruise -> Pb)');

% 1. Position
subplot(4, 1, 1);
plot(time_vec, p_hist, 'LineWidth', 1); grid on;
ylabel('Pos (m)');
if tpb < plot_time
xline(tpb, 'k--', 'Pa');
end
if tc > 1e-9 && (tpb + tc) < plot_time
    xline(tpb+tc, 'k--', 'Cruise');
end
if Pb.T3 > 1e-9 && (tpb + tc + Pb.T3) < plot_time
    xline(tpb+tc+Pb.T3, 'r--', 'Pb');
end
yline(target_pos, 'r--', 'Target Pos');


% 2. Velocity
subplot(4, 1, 2);
plot(time_vec, v_hist, 'LineWidth', 1); grid on;
ylabel('Vel (m/s)');

% 3. Acceleration
subplot(4, 1, 3);
plot(time_vec, a_hist, 'LineWidth', 1); grid on;
ylabel('Acc (m/s^2)');
if nargin >= 5
    yline(limits.a_max, 'k:'); yline(limits.a_min, 'k:');
end

% 4. Jerk
subplot(4, 1, 4);
stairs(time_vec, j_hist, 'LineWidth', 1); grid on;
ylabel('Jerk (m/s^3)');
xlabel('Time (s)');
if nargin >= 5
    yline(limits.j_max, 'k:'); yline(limits.j_min, 'k:');
end
end