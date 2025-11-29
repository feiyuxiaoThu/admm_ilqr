function [p, v, a] = evaluate_position(S, v0, a0, p0, t)
    % 处理 target_position_solver 的复合轨迹
    % S 包含: Pa, Pb, tpb, tc
    
    % 1. 加速阶段 (Pa)
    if t <= S.tpb
        [p, v, a] = get_state_at_t(v0, a0, p0, S.Pa, t);
        return;
    end
    
    % 计算第一阶段结束状态 (作为后续阶段的基础)
    % 注意：这里需要重新计算一次 endpoint，或者在 S 中存储这些中间状态以加速
    [ppb, vpb, apb] = get_state_at_t(v0, a0, p0, S.Pa, S.tpb);
    
    % 2. 巡航阶段 (Cruise)
    if t <= (S.tpb + S.tc)
        dt_cruise = t - S.tpb;
        a = 0; 
        v = vpb; % 匀速
        p = ppb + v * dt_cruise;
        return;
    end
    
    % 计算巡航结束状态
    p_end_cruise = ppb + vpb * S.tc;
    
    % 3. 减速阶段 (Pb)
    if S.tc > 0
        a_start_b = 0; % 如果有巡航，Pb 从 0 加速度开始
    else
        a_start_b = apb; % 否则继承 Pa 的加速度
    end
    t_in_b = t - (S.tpb + S.tc);
    if t_in_b > S.Pb.T3
        % 保持最终状态 (v=0, a=0)
        [p_final, v_final, a_final] = get_state_at_t(vpb, a_start_b, p_end_cruise, S.Pb, S.Pb.T3);
        p = p_final; v = v_final; a = a_final; 
    else
        [p, v, a] = get_state_at_t(vpb, a_start_b, p_end_cruise, S.Pb, t_in_b);
    end
end