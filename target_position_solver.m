function [Pa, Pb, tpb, tc, total_time] = target_position_solver(p0, v0, a0, pf, limits, vmax, vmin)
    % 输入: 状态 p0, v0, a0, 目标 pf, 限制 limits, 速度边界 vmax/vmin
    % 输出: Pa (加速段), Pb (减速段), tpb (切换时间点), tc (巡航时间)
    
    % --- Line 3: 计算完全停车轨迹 P ---
    P = target_velocity_solver(v0, a0, 0, limits);
    
    % --- Line 4: 获取停车点状态 (psp, vsp, asp) ---
    [psp, ~, ~] = get_state_at_t(v0, a0, p0, P, P.T3);
    
    % --- Line 5: 计算位置方向 dp ---
    dp = sign(pf - psp);
    if abs(pf - psp) < 1e-9, dp = 0; end
    
    % --- Line 6-12: 确定巡航速度 vc ---
    if dp == 1
        vc = vmax;
    elseif dp == -1
        vc = vmin;
    else
        vc = 0;
    end
    
    % --- Line 13: 计算加速到 vc 的轨迹 Pa ---
    Pa = target_velocity_solver(v0, a0, vc, limits);
    
    % --- Line 14: 获取 Pa 终点状态 (pfa, v, a) ---
    [pfa, v_actual_cruise, a_end_Pa] = get_state_at_t(v0, a0, p0, Pa, Pa.T3);
    
    % --- Line 15: 计算从 vc 刹停的轨迹 Pb (注意初始速度为 vc, 初始加速度为 0) ---
    Pb = target_velocity_solver(v_actual_cruise, 0, 0, limits);
    
    % --- Line 16: 获取 Pb 终点状态 pfb (起始位置是 pfa) ---
    [pfb, ~, ~] = get_state_at_t(v_actual_cruise, 0, pfa, Pb, Pb.T3);
    
    % --- Line 18: 判断是否欠冲 (Need Cruise) ---
    % 逻辑: 如果 (最终位置 - 目标) 与 dp 异号，说明还没跑到
    if sign(pfb - pf) * dp <= 0
        % --- Line 19: 计算巡航时间 tc ---
        if abs(v_actual_cruise) > 1e-9
            tc = (pf - pfb) / v_actual_cruise;
        else
            tc = 0;
        end
        % --- Line 20: 设定切换时间 tpb 为 Pa 的全长 ---
        tpb = Pa.T3;
        
    else
        % --- Line 21-24: 过冲 (Overshoot)，需要二分查找 ---
        tc = 0;
        tH = Pa.T3; % 上界
        tL = 0;     % 下界
        
        N = 50; % 迭代次数
        epsilon = 1e-3; % 收敛阈值
        
        % --- Line 25: 循环 ---
        for counter = 1:N
            % --- Line 26: 二分时间 tpb ---
            tpb = (tH + tL) / 2;
            
            % --- Line 27: 获取 Pa 在 tpb 时刻的状态 (ppb, vpb, apb) ---
            [ppb, vpb, apb] = get_state_at_t(v0, a0, p0, Pa, tpb);
            
            % --- Line 28: 基于切断点的状态，重新计算刹停轨迹 Pb ---
            % 目标速度是 0
            Pb = target_velocity_solver(vpb, apb, 0, limits);
            
            % --- Line 29: 计算新的最终位置 pfb (起始位置是 ppb) ---
            [pfb, ~, ~] = get_state_at_t(vpb, apb, ppb, Pb, Pb.T3);
            
            % --- Line 30-34: 更新二分界限 ---
            % sign(pfb - pf) * dp < 0 意味着我们跑得不够远 (Undershoot)，
            % 所以需要更长的加速时间 -> tL = tpb
            if sign(pfb - pf) * dp < 0
                tL = tpb;
            else
                tH = tpb;
            end
            
            % --- Line 35-37: 检查收敛 ---
            if abs(pfb - pf) < epsilon
                break;
            end
        end
    end
    
    total_time = tpb + tc + Pb.T3;
end