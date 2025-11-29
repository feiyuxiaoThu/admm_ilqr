function [p, v, a] = get_state_at_t(v0, a0, p0, P, t_query)
    % 严格按照伪代码中的 getState 实现
    % 输入: 初始状态 (v0, a0, p0), 轨迹 P, 查询时刻 t_query
    
    % 为了方便计算，使用 duration
    dur_1 = P.T1;
    dur_2 = P.T2 - P.T1;
    dur_3 = P.T3 - P.T2; 
    
    p = p0; v = v0; a = a0;
    t_rem = t_query;
    
    % Phase 1
    if t_rem > 0
        dt = min(t_rem, dur_1);
        p = p + v*dt + 0.5*a*dt^2 + (1/6)*P.j1*dt^3;
        v = v + a*dt + 0.5*P.j1*dt^2;
        a = a + P.j1*dt;
        t_rem = t_rem - dt;
    end
    
    % Phase 2
    if t_rem > 0
        dt = min(t_rem, dur_2);
        p = p + v*dt + 0.5*a*dt^2 + (1/6)*P.j2*dt^3; % P.j2 should be 0
        v = v + a*dt + 0.5*P.j2*dt^2;
        a = a + P.j2*dt;
        t_rem = t_rem - dt;
    end
    
    % Phase 3
    if t_rem > 0
        % 剩下的时间都在 Phase 3，不需要上限，因为传入的 t_query 通常就在 P.T3 内
        % 但如果是 getState(..., P.T3) 可能会有微小浮点误差，直接用 t_rem 即可
        dt = min(t_rem, dur_3);
        p = p + v*dt + 0.5*a*dt^2 + (1/6)*P.j3*dt^3;
        v = v + a*dt + 0.5*P.j3*dt^2;
        a = a + P.j3*dt;
        t_rem = t_rem - dt;
    end

    % 如果 t_rem > 0，说明已经超过轨迹时间，保持末状态不变
    if t_rem > 0
        p = p + v * t_rem;
        % v, a 不变
    end
end