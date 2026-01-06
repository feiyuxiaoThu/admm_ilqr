clc; clear;
close all;

% Global parameters
dt = 0.1;           % time step (s)
L = 3.0;            % wheelbase (m)
v_target = 10.0;    % target speed (m/s)
total_time = 6.0;   % total simulation time (s)
N = floor(total_time / dt);

% iLQR optimization weights
weights.q_pos_x = 0.1;    % position x penalty (stage)
weights.q_pos_y = 0.2;    % position y penalty (stage)
weights.q_vel = 0.0;      % velocity penalty (stage)
weights.r_acc = 0.5;      % acceleration penalty
weights.r_steer = 50.0;   % steering penalty
weights.q_pos_x_term = 0.1;   % terminal x penalty
weights.q_pos_y_term = 50.0;  % terminal y penalty
weights.q_vel_term = 0.0;     % terminal velocity penalty
weights.r_delta_acc = 10.0;    % control rate penalty (acceleration)
weights.r_delta_steer = 10.0;  % control rate penalty (steering)

% Trajectory evaluation weights (for selecting best candidate)
% Note: Different from iLQR internal weights - these are decision-level weights
eval_weights.w_safety      = 2.0;    % safety margin penalty
eval_weights.w_progress    = 10.0;   % progress reward
eval_weights.w_ref_vel     = 0.5;    % velocity error penalty
eval_weights.w_acc         = 0.1;    % acceleration penalty (comfort)
eval_weights.w_steer       = 100.0;  % steering penalty
eval_weights.w_jerk_lon    = 0.1;    % longitudinal jerk penalty
eval_weights.w_jerk_lat    = 100.0;  % lateral jerk penalty
eval_weights.w_consistency = 500.0;  % decision consistency (avoid frequent switching)
last_best_id = -1;  % Initialize best candidate from previous frame

% ADMM-iLQR algorithm parameters
options.max_admm_iter = 10;     % max ADMM iterations
options.sigma = 10.0;           % ADMM penalty parameter (initial)
options.tol_admm = 1e-1;        % ADMM convergence tolerance
options.alpha_or = 1.0;         % ADMM over-relaxation parameter
options.adjust_sigma = false;   % adaptive sigma adjustment
options.max_ilqr_iter = 50;     % max iLQR iterations per ADMM step
options.ilqr_tol = 1e-1;        % iLQR convergence tolerance

% Build simulation scenario
[scenario, constraints, x0] = build_simulation_scenario(dt, N);

% Generate candidate trajectories
candidates = generate_candidates_structured(x0, scenario, constraints, N, dt);
fprintf('Generated %d candidate trajectories.\n', length(candidates));

% Optimize all candidates in parallel using ADMM-iLQR
results = struct();
tic;
% parfor i = 1:length(candidates)  % Enable parallel processing
for i = 1:length(candidates)    % Sequential version for debugging
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

% % Optional: Plot iLQR iteration history for each candidate
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

% Evaluate trajectories and select best candidate
scenario_params.v_desired = scenario.v_desired;  % desired speed
[best_idx, best_score, all_scores] = evaluate_trajectories(...
    results, ...
    constraints.obstacles, ...
    last_best_id, ...
    scenario_params, ...
    eval_weights);

% Output decision result
if best_idx ~= -1
    best_cand = results(best_idx).cand;
    fprintf('\n>>> 🌟 FINAL DECISION: Candidate %d (%s) \n', best_cand.id, best_cand.name);
    fprintf('    Score: %.4f (Safe:%.1f, Prog:%.1f, Comf:%.1f)\n', ...
        best_score, ...
        all_scores(best_idx).J_safe * eval_weights.w_safety, ...
        all_scores(best_idx).J_prog, ...
        all_scores(best_idx).J_comf);
    
    % Update best ID for next frame consistency check
    last_best_id = best_cand.id;
    
    % Extract optimal trajectory for execution
    final_X = results(best_idx).X;
    final_U = results(best_idx).U;
else
    warning('EMERGENCY: No valid trajectory found! Triggering AEB.');
end

% Visualization
if best_idx ~= -1
    plot_results_multimodal(results, best_idx, constraints, scenario, dt);
else
    warning('No valid trajectory to plot.');
end
