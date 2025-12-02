% clear environment
clc; clear;
close all;

% 1. Global parameters
dt = 0.1;           % time step (s)
L = 3.0;            % wheelbase (m)
v_target = 10.0;    % target speed (m/s)
total_time = 6.0;   % total simulation time (s)
N = floor(total_time / dt);

% Weights
weights.q_pos_x = 0.1;    % position x penalty
weights.q_pos_y = 0.2;    % position y penalty
weights.q_vel = 0.0;    % velocity penalty
weights.r_acc = 0.5;    % acceleration penalty
weights.r_steer = 50.0; % steering penalty
weights.q_pos_x_term = 0.1; % X position terminal penalty
weights.q_pos_y_term = 50.0; % Y position terminal penalty
weights.q_vel_term = 0.0;  % velocity terminal penalty
weights.r_delta_acc = 10.0;
weights.r_delta_steer = 10.0;

% 轨迹评估权重 (Evaluator Weights)
% 这些权重用于从并行优化的结果中选出最佳轨迹
% 注意：这与 iLQR 内部的优化权重不同，这是决策层的权重
eval_weights.w_safety      = 2.0;  % 安全裕度权重 (很高，哪怕侵入一点警戒区也要重罚)
eval_weights.w_progress    = 10.0;   % 行驶距离奖励 (越大越倾向于跑得快)
eval_weights.w_ref_vel     = 0.5;   % 速度误差惩罚 (越大越倾向于维持限速)
eval_weights.w_acc         = 0.1;   % 加速度惩罚 (舒适性)
eval_weights.w_steer       = 100.0;   % 转向惩罚
eval_weights.w_jerk_lon    = 0.1;
eval_weights.w_jerk_lat    = 100.0;
eval_weights.w_consistency = 500.0;   % 决策一致性 (防止在 Keep 和 Change 之间频繁跳变)
% 初始化上一帧的最佳 ID
last_best_id = -1;

% ADMM-iLQR parameters
options.max_admm_iter = 10;     % ADMM 最大迭代次数
options.sigma = 10.0;           % ADMM 惩罚参数 (初始)
options.tol_admm = 1e-1;        % ADMM 收敛容差
options.alpha_or = 1.0;        % ADMM 过松弛参数，1.0 表示无过松弛 (ADMM 收敛容差较大时（1e-1）不需要改)
options.adjust_sigma = false;   % 是否自适应调整 sigma (开启收敛更慢)
options.max_ilqr_iter = 50;     % 内部 iLQR 最大迭代次数
options.ilqr_tol = 1e-1;        % iLQR 收敛容差

% 2. Build simulation scenario
[scenario, constraints, x0] = build_simulation_scenario(dt, N);

% 3. Generate candidate goals
candidates = generate_candidates_structured(x0, scenario, constraints, N, dt);
fprintf('Generated %d candidate trajectories.\n', length(candidates));

% 4. Multi-branch ADMM-iLQR
results = struct();
tic;
parfor i = 1:length(candidates) % 开启并行
% for i = 1:length(candidates)
    fprintf('Optimizing Candidate %d: %s (Target V=%.2f)\n', ...
        candidates(i).id, candidates(i).name, candidates(i).v_target);
    % tic;
    [X_opt, U_opt, debug_info] = run_admm_ilqr(x0, candidates(i), constraints, dt, L, weights, options);
    % elapsed = toc;
    % fprintf('  Candidate %d optimized in %.4f s. Final Cost: %.4f\n', ...
    %     candidates(i).id, elapsed, debug_info.cost_history(end));
    results(i).X = X_opt;
    results(i).U = U_opt;
    results(i).cost = debug_info.cost_history(end);
    results(i).cand = candidates(i);
    results(i).debug_info = debug_info;
    % results(i).time_cost = elapsed;
end
total_time = toc;
fprintf('All candidates optimized. Total time: %.4f s\n', total_time);

% for i = 1:length(results)
%     fprintf('Candidate %d: %s, Final Cost: %.4f\n', ...
%         results(i).cand.id, results(i).cand.name, results(i).cost);
%     plot_ilqr_iter = false;
%     fig_plot = figure('Name', 'iLQR iteration history', 'NumberTitle', 'off');
%     if exist('fig_plot','var')
%         plot_iteration(fig_plot, dt, results(i).debug_info, constraints, candidates(i).x_ref, plot_ilqr_iter, scenario.ego_size);
%     end
%     % plot_results(results(i).X, results(i).U, candidates(i).x_ref, dt, candidates(i).id, constraints);

%     % close all;
% end

% 5. Evaluate trajectories and select the best one
% 构造场景参数供评估器使用
scenario_params.v_desired = scenario.v_desired; % 例如 10.0 m/s

% 调用评估器
[best_idx, best_score, all_scores] = evaluate_trajectories(...
    results, ...
    constraints.obstacles, ...
    last_best_id, ...
    scenario_params, ...
    eval_weights);

% 输出结果
if best_idx ~= -1
    best_cand = results(best_idx).cand;
    fprintf('\n>>> 🌟 FINAL DECISION: Candidate %d (%s) \n', best_cand.id, best_cand.name);
    fprintf('    Score: %.4f (Safe:%.1f, Prog:%.1f, Comf:%.1f)\n', ...
        best_score, ...
        all_scores(best_idx).J_safe * eval_weights.w_safety, ...
        all_scores(best_idx).J_prog, ...
        all_scores(best_idx).J_comf);
        
    % 更新上一帧 ID (用于下一帧的一致性计算)
    last_best_id = best_cand.id;
    
    % 提取最终要执行的轨迹
    final_X = results(best_idx).X;
    final_U = results(best_idx).U;
    
    % 这里可以加一个绘图函数，画出所有候选轨迹，并高亮最佳轨迹
else
    warning('EMERGENCY: No valid trajectory found! Triggering AEB.');
    % 触发紧急制动逻辑 (AEB)
end

% 可视化
if best_idx ~= -1
    plot_results_multimodal(results, best_idx, constraints, scenario, dt);
else
    warning('No valid trajectory to plot.');
end
