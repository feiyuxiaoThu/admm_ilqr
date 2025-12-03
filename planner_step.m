function [u_cmd, plan_info] = planner_step(x_curr, obstacles, scenario, last_plan_info, params)
% PLANNER_STEP - Single-frame motion planning interface
%
% Workflow: candidate generation -> parallel optimization -> evaluation and selection
%
% Inputs:
%   x_curr: current state [X; Y; phi; v] (4x1)
%   obstacles: current perceived obstacles (struct array)
%   scenario: scenario parameters (struct)
%   last_plan_info: previous frame planning results (for warm start)
%   params: planning parameters (struct with dt, N, L, constraints, weights, options, eval_weights)
%
% Outputs:
%   u_cmd: control input to execute [a; delta] (2x1)
%   plan_info: planning information for next frame and logging

% Unpack parameters
dt = params.dt;
N = params.N;
L = params.L;
constraints = params.constraints;
constraints.obstacles = obstacles;  % Update with current perception

weights = params.weights;
options = params.options;
eval_weights = params.eval_weights;

% Generate candidate maneuvers
candidates = generate_candidates_structured(x_curr, scenario, constraints, N, dt);

% Prepare warm start data using previous optimal trajectory
warm_start_map = containers.Map();  % Map to store <CandidateName, U_traj>

if ~isempty(last_plan_info) && last_plan_info.best_idx ~= -1
    % Retrieve optimal control sequence from previous frame
    last_U = last_plan_info.final_traj_U;  % 2 x N

    % Shift operation: U(k) <- U(k+1), U(N) <- U(N) (Zero Order Hold)
    U_warm = [last_U(:, 2:end), last_U(:, end)];

    % Store with candidate name for warm start matching
    last_name = last_plan_info.best_cand_name;

    % Simple name matching logic: warm start only for exact name matches
    % Note: generated candidate names are e.g., 'Keep_Cruise', 'Left_Stop'
    % During lane change, candidate names may change (e.g., from 'Left_Cruise' to 'Keep_Cruise')
    % For simplicity, we assume candidate names are stable over short durations

    warm_start_map(last_name) = U_warm;
end

% Parallel optimization with ADMM-iLQR
results = struct();
% Pre-allocate results structure
for i = 1:length(candidates)
    results(i).cand = candidates(i);
    results(i).X = [];
    results(i).U = [];
    results(i).cost = inf;
    results(i).valid = false;
end

% Optimize all candidates in parallel
parfor i = 1:length(candidates)
    cand = candidates(i);
    % Check for warm start data
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
        % Check for divergence (NaN check)
        if any(isnan(X_opt(:))) || any(isnan(U_opt(:)))
            results(i).valid = false;
            results(i).cost = inf;
        else
            results(i).cost = debug_info.cost_history(end);
            results(i).valid = true;
        end
        % results(i).debug = debug_info;  % Reduce data size
    catch
        results(i).cand = cand;
        results(i).valid = false;
    end
end

% Evaluate and select best trajectory

% Retrieve previous best target lane for consistency
if isempty(last_plan_info) || ~isfield(last_plan_info, 'best_target_lane_id')
    last_best_target_lane_id = -1;
else
    last_best_target_lane_id = last_plan_info.best_target_lane_id;
end

% Construct scenario parameters for evaluation
scenario_params.v_desired = scenario.v_desired;

[best_idx, best_score, all_scores] = evaluate_trajectories(...
    results, obstacles, last_best_target_lane_id, scenario_params, eval_weights);

% Package output for next frame and logging
plan_info.timestamp = now;  % Timestamp for this planning cycle
plan_info.results = results;
plan_info.best_idx = best_idx;
plan_info.best_score = best_score;

if best_idx ~= -1
    % Optimal solution found
    best_cand = results(best_idx).cand;
    plan_info.best_cand_id = best_cand.id;
    plan_info.best_cand_name = best_cand.name;
    plan_info.best_target_lane_id = best_cand.target_lane_id;
    plan_info.final_traj_X = results(best_idx).X;
    plan_info.final_traj_U = results(best_idx).U;

    % Extract first control input for execution
    u_cmd = results(best_idx).U(:, 1);

    % Apply saturation as final safety check
    u_cmd = max(constraints.u_min, min(constraints.u_max, u_cmd));
else
    % Failsafe: no valid trajectory found, execute emergency brake
    warning('Planner failed to find valid trajectory. Executing Emergency Brake.');

    acc_brake = constraints.u_min(1);  % Maximum braking
    steer_hold = 0;  % Zero steering
    u_cmd = [acc_brake; steer_hold];

    plan_info.best_cand_id = -1;
    plan_info.best_cand_name = 'EMERGENCY_BRAKE';
    plan_info.best_target_lane_id = -1;
    % Create dummy brake trajectory for display
    plan_info.final_traj_X = repmat(x_curr, 1, N+1);
    plan_info.final_traj_U = zeros(2, N);
end
end