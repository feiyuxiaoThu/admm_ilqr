function [u_cmd, plan_info] = planner_step(x_curr, obstacles, scenario, last_plan_info, params)
% PLANNER_STEP 单帧规划接口
% 执行流程: 生成候选 -> 并行优化 -> 评估择优

% 1. 解包参数
dt = params.dt;
N = params.N;
L = params.L;
constraints = params.constraints;
constraints.obstacles = obstacles; % 更新当前感知的障碍物

weights = params.weights;
options = params.options;
eval_weights = params.eval_weights;

% 2. 生成候选意图 (Candidate Generation)
candidates = generate_candidates_structured(x_curr, scenario, constraints, N, dt);

% 3. 准备热启动数据 (Shift Logic)
warm_start_map = containers.Map(); % 使用 Map 存储 <CandidateName, U_traj>

if ~isempty(last_plan_info) && last_plan_info.best_idx ~= -1
    % 获取上一帧的最优控制序列
    last_U = last_plan_info.final_traj_U; % 2 x N

    % 平移操作 (Shift)
    % U(k) <- U(k+1)
    % U(N) <- U(N) (Zero Order Hold)
    U_warm = [last_U(:, 2:end), last_U(:, end)];

    % 绑定到对应的意图名称
    % 只有当前生成的 candidates 中名称与上一帧最优名称一致的，才使用热启动
    last_name = last_plan_info.best_cand_name;

    % 简单的名称匹配逻辑
    % 注意：generate_candidates_structured 生成的名称如 'Keep_Cruise', 'Left_Stop'
    % 如果我们处于变道过程中，下一帧的名称可能会变（例如从 Left_Cruise 变成 Keep_Cruise，因为已经到了左车道）
    % 但为了简单起见，我们先假设名称在短时间内是稳定的，或者只热启动完全匹配的。

    warm_start_map(last_name) = U_warm;
end

% 4. 并行优化 (Parallel Optimization)
results = struct();
% 注意: 在函数内部无法直接用 parfor 更新 struct array 的不同字段
% 需要先用临时变量或者规整的切片。这里为了简单先用 for，
% 如果追求性能，可以将 run_admm_ilqr 的输出对齐后用 parfor。
% 预分配
for i = 1:length(candidates)
    results(i).cand = candidates(i);
    results(i).X = [];
    results(i).U = [];
    results(i).cost = inf;
    results(i).valid = false;
end

% 开启并行 (如果环境支持)
parfor i = 1:length(candidates)
    cand = candidates(i);
    % 检查是否有热启动数据
    U_guess = [];
    if isKey(warm_start_map, cand.name)
        U_guess = warm_start_map(cand.name);
        % fprintf('Warm starting candidate: %s\n', cand.name);
    end

    try
        [X_opt, U_opt, debug_info] = run_admm_ilqr(x_curr, cand, constraints, dt, L, weights, options, U_guess);

        results(i).cand = cand;
        results(i).X = X_opt;
        results(i).U = U_opt;
        % 检查 iLQR 是否发散 (NaN check)
        if any(isnan(X_opt(:))) || any(isnan(U_opt(:)))
            results(i).valid = false;
            results(i).cost = inf;
        else
            results(i).cost = debug_info.cost_history(end);
            results(i).valid = true;
        end
        % results(i).debug = debug_info; % 减少数据量，暂不存
    catch
        results(i).cand = cand;
        results(i).valid = false;
    end
end

% 5. 评估与择优 (Evaluation & Selection)

% 获取上一帧的最佳 ID
if isempty(last_plan_info) || ~isfield(last_plan_info, 'best_target_lane_id')
    last_best_target_lane_id = -1;
else
    last_best_target_lane_id = last_plan_info.best_target_lane_id;
end

% 构造场景参数
scenario_params.v_desired = scenario.v_desired;

[best_idx, best_score, all_scores] = evaluate_trajectories(...
    results, obstacles, last_best_target_lane_id, scenario_params, eval_weights);

% 6. 封装输出
plan_info.timestamp = now; % 或传入的 sim_time
plan_info.results = results;
plan_info.best_idx = best_idx;
plan_info.best_score = best_score;

if best_idx ~= -1
    % 找到最优解
    best_cand = results(best_idx).cand;
    plan_info.best_cand_id = best_cand.id;
    plan_info.best_cand_name = best_cand.name;
    plan_info.best_target_lane_id = best_cand.target_lane_id;
    plan_info.final_traj_X = results(best_idx).X;
    plan_info.final_traj_U = results(best_idx).U;

    % 提取第一个控制量用于执行
    u_cmd = results(best_idx).U(:, 1);

    % [Safety Check] 防止由数值误差导致的突然大转向
    % u_cmd = clip... (在 update_state 里通常不会做 clip，这里可以做最后一道防线)
    u_cmd = max(constraints.u_min, min(constraints.u_max, u_cmd));
else
    % Failsafe: 没有可行解，执行最大制动
    % 保持方向盘不变 (或回正)，全力刹车
    warning('Planner failed to find valid trajectory. Executing Emergency Brake.');

    acc_brake = constraints.u_min(1); % 最大刹车
    steer_hold = 0; % 或者保持上一帧 steer
    u_cmd = [acc_brake; steer_hold];

    plan_info.best_cand_id = -1;
    plan_info.best_cand_name = 'EMERGENCY_BRAKE';
    plan_info.best_target_lane_id = -1;
    % 生成一条假想的刹车轨迹用于显示
    plan_info.final_traj_X = repmat(x_curr, 1, N+1);
    plan_info.final_traj_U = repmat(u_cmd, 1, N);
    % 补全缺失的 final_traj_U 字段，防止下一帧热启动报错
    plan_info.final_traj_U = zeros(2, N);
end
end