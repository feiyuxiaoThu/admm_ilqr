function P = target_velocity_solver(v0, a0, v_d, limits)
% 输入: v0, a0, v_d (目标速度)
%       limits 包含: amax, amin, jmax, jmin
% 输出: P (包含累积时间节点 T1, T2, T3 和加加速度 j1, j2, j3)

amax = limits.a_max; amin = limits.a_min;
jmax = limits.j_max; jmin = limits.j_min;

% --- Line 3-7: 计算预估速度 ve ---
if a0 >= 0
    ve = v0 + a0 * abs(a0/jmin) / 2;
else
    ve = v0 + a0 * abs(a0/jmax) / 2;
end

% --- Line 8: 计算方向 da ---
da = sign(v_d - ve);
if abs(v_d - ve) < 1e-9, da = 0; end % 浮点数零处理

% --- Line 9-15: 确定巡航加速度 ac ---
if da == 1
    ac = amax;
elseif da == -1
    ac = amin;
else
    ac = 0;
end

% --- Line 16-22: 计算第一阶段 t1, j1 ---
if (ac - a0) > 0
    t1 = (ac - a0)/jmax;
    j1 = jmax;
elseif (ac - a0) < 0
    t1 = (ac - a0)/jmin;
    j1 = jmin;
else
    t1 = 0;
    j1 = 0; % no change in acceleration needed
end

% --- Line 23: 计算 t1 时刻的速度 v1 ---
v1 = v0 + a0*t1 + 0.5*j1*t1^2;

% --- Line 24-30: 计算第三阶段 t3, j3 ---
if -ac > 0
    t3 = -ac/jmax;
    j3 = jmax;
elseif -ac < 0
    t3 = -ac/jmin;
    j3 = jmin;
else
    t3 = 0;
    j3 = 0; % no change in acceleration needed
end

% --- Line 31: 计算 t3 阶段的速度变化量 v3_bar ---
% 注意：原文公式是 v3 = ac*t3 + t3^2*j3/2
v3_bar = ac*t3 + 0.5*j3*t3^2;

% --- Line 32: 计算剩余需要的速度变化量 v2_bar ---
v2_bar = v_d - v1 - v3_bar;

% --- Line 33-37: 计算匀加速段时间 t2 ---
if da == 0
    t2 = 0;
elseif abs(ac) < 1e-9  % 【修复】增加除零保护
    % 如果需要加速(da!=0)但物理限制为0(ac=0)，说明无法加速。
    % 此时检查是否是因“过冲”导致的特殊情况（虽然ac=0，但可能v2_bar反向？）：
    if (v2_bar * da < 0)
        t2 = -1; % 强制触发后续的过冲修正逻辑 (Solver logic)
    else
        t2 = 0;  % 无法加速，只能保持当前状态 (Best Effort)
    end
else
    t2 = v2_bar / ac;
end

% --- Line 38: 检查 t2 是否小于 0 (过冲处理) ---
if t2 < 0
    if da == 1
        % --- Line 40-43: 加速过冲，修正 an ---
        % 公式: sqrt( (2*(vd-v0) + a0^2/jmax) / (1/jmax - 1/jmin) )
        numerator = 2*(v_d - v0) + a0^2/jmax;
        denominator = 1/jmax - 1/jmin;
        an = sqrt(numerator / denominator);

        t1 = (an - a0)/jmax;
        t2 = 0;
        t3 = -an/jmin;

        % 更新 j1, j3 以匹配修正后的轨迹
        j1 = jmax;
        j3 = jmin;

    elseif da == -1
        % --- Line 45-48: 减速过冲，修正 an ---
        % 公式: -sqrt( (2*(vd-v0) + a0^2/jmin) / (1/jmin - 1/jmax) )
        numerator = 2*(v_d - v0) + a0^2/jmin;
        denominator = 1/jmin - 1/jmax;
        an = -sqrt(numerator / denominator);

        t1 = (an - a0)/jmin;
        t2 = 0;
        t3 = -an/jmax;

        % 更新 j1, j3
        j1 = jmin;
        j3 = jmax;
    end
end

% --- Line 51-56: 构建输出 P (累积时间) ---
P.T1 = t1;
P.T2 = t1 + t2;
P.T3 = t1 + t2 + t3; % 总时间
P.j1 = j1;
P.j2 = 0;
P.j3 = j3;

% 额外存储各段时间长度，方便 getState 使用，虽然伪代码没写，但物理上必须知道
P.t1 = t1; P.t2 = t2; P.t3 = t3;
end