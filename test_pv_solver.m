% --- 修正后的验证与绘图脚本 ---
close all; clear; clc;

% 1. 重新运行计算
p0 = 0; v0 = 8; a0 = 0; pf = 60;
limits.a_max = 2.0; limits.a_min = -3.0; 
limits.j_max = 1.0; limits.j_min = -1.0;
v_max = 15; v_min = 0;

[Pa, Pb, tpb, tc, T_total] = target_position_solver(p0, v0, a0, pf, limits, v_max, v_min);

fprintf('修正后计算: 总时间=%.4fs, 切换点tpb=%.4fs\n', T_total, tpb);

% 2. 采样与绘图
dt = 0.01;
time_vec = 0:dt:(T_total + 0.5); % 多画一点看末端稳态
s_hist = []; v_hist = []; a_hist = []; t_hist = [];

for t = time_vec
    if t <= tpb
        % === 第一阶段：Pa (加速/调整) ===
        [s, v, a] = get_state_at_t(v0, a0, p0, Pa, t);
    elseif t <= tpb + tc
        % === 第二阶段：巡航 (Cruise) ===
        % 获取 Pa 结束时的状态
        [s_end_a, v_end_a, a_end_a] = get_state_at_t(v0, a0, p0, Pa, tpb);
        
        t_cruise = t - tpb;
        s = s_end_a + v_end_a * t_cruise;
        v = v_end_a;
        a = 0; % 巡航时加速度为 0
    else
        % === 第三阶段：Pb (刹车/入库) ===
        % 1. 获取 Pa 结束状态
        [s_end_a, v_end_a, a_end_a] = get_state_at_t(v0, a0, p0, Pa, tpb);
        
        % 2. 计算巡航结束后的状态 (作为 Pb 的起点)
        s_start_b = s_end_a + v_end_a * tc;
        v_start_b = v_end_a;
        
        % 关键修正：Pb 的初始加速度取决于是否有巡航
        if tc > 0
            a_start_b = 0; % 如果有巡航，则从 0 加速度开始刹车
        else
            a_start_b = a_end_a; % 【重要】如果是二分截断，必须继承 Pa 的加速度！
        end
        
        t_in_b = t - (tpb + tc);
        
        % 3. 积分 Pb (防止超出 Pb 规划的时间导致数值外推)
        if t_in_b > Pb.T3
             % 保持最终状态 (v=0, a=0)
             [s_final, v_final, a_final] = get_state_at_t(v_start_b, a_start_b, s_start_b, Pb, Pb.T3);
             s = s_final; v = 0; a = 0; 
        else
             [s, v, a] = get_state_at_t(v_start_b, a_start_b, s_start_b, Pb, t_in_b);
        end
    end
    
    s_hist(end+1) = s;
    v_hist(end+1) = v;
    a_hist(end+1) = a;
    t_hist(end+1) = t;
end

% 绘图
figure('Name', 'Corrected Trajectory');
subplot(3,1,1); plot(t_hist, s_hist, 'LineWidth', 1.5); 
hold on; yline(pf, 'r--', 'Target'); 
ylabel('Position (m)'); grid on; title(sprintf('Target: %.1fm, Final: %.2fm', pf, s_hist(end)));
xline(tpb, 'k--', 'Pa');
if tc > 1e-9
    xline(tpb+tc, 'k--', 'Cruise');
end
if Pb.T3 > 1e-9
    xline(tpb+tc+Pb.T3, 'r--', 'Pb');
end

subplot(3,1,2); plot(t_hist, v_hist, 'LineWidth', 1.5); 
ylabel('Velocity (m/s)'); grid on; 

subplot(3,1,3); plot(t_hist, a_hist, 'LineWidth', 1.5); 
ylabel('Accel (m/s^2)'); grid on; xlabel('Time (s)');