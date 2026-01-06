% run_closed_loop_simulation.m
% Closed-loop simulation main program: Physics update -> Traffic flow -> Perception -> Planning -> Control -> Logging

clear; clc; close all;

% ==========================================
% 1. Simulation Configuration
% ==========================================
T_sim = 40.0;       % Total simulation duration (s) -> increase to test infinite flow
dt = 0.1;           % Simulation timestep (s)
N_steps = floor(T_sim / dt);

% Planner parameters
planner_params.dt = dt;
planner_params.N = 60;  % Prediction horizon steps (6.0s)
planner_params.L = 3.0; % Wheelbase

% Weight configuration (consistent with main_ADMM_iLQR.m)
planner_params.weights.q_pos_x = 0.1;
planner_params.weights.q_pos_y = 0.2;
planner_params.weights.q_vel = 0.0;
planner_params.weights.r_acc = 0.5;
planner_params.weights.r_steer = 50.0;
planner_params.weights.q_pos_x_term = 0.1;
planner_params.weights.q_pos_y_term = 50.0;
planner_params.weights.q_vel_term = 0.0;
planner_params.weights.r_delta_acc = 10.0;
planner_params.weights.r_delta_steer = 10.0;

% Evaluation weights
planner_params.eval_weights.w_safety      = 2.0;
planner_params.eval_weights.w_progress    = 10.0;
planner_params.eval_weights.w_ref_vel     = 0.5;
planner_params.eval_weights.w_acc         = 0.1;
planner_params.eval_weights.w_steer       = 100.0;
planner_params.eval_weights.w_jerk_lon    = 0.1;
planner_params.eval_weights.w_jerk_lat    = 100.0;
planner_params.eval_weights.w_consistency = 500.0;

% Solver options
planner_params.options.max_admm_iter = 10;
planner_params.options.sigma = 10.0;
planner_params.options.tol_admm = 1e-1;
planner_params.options.alpha_or = 1.0;
planner_params.options.adjust_sigma = false;
planner_params.options.max_ilqr_iter = 20;
planner_params.options.ilqr_tol = 1e-1;

% ==========================================
% 2. Environment Initialization
% ==========================================
% Build static scenario and initial obstacles
[scenario, constraints_init, x0, raw_obs_list] = build_simulation_scenario(planner_params.dt, planner_params.N);
planner_params.constraints = constraints_init; 

% Ego vehicle state initialization [x; y; theta; v]
x_curr = x0;
last_plan_info = [];

% Data logging
sim_log = struct();

fprintf('Simulation Started... Total Steps: %d\n', N_steps);
h_wait = waitbar(0, 'Simulation in progress...');

% ==========================================
% 3. Simulation Main Loop
% ==========================================
for k = 1:N_steps
    sim_time = (k-1) * dt;
    
    % --- Step A: Physics Update ---
    % 1. Update obstacle dynamics
    raw_obs_list = update_obstacle_physics(raw_obs_list, dt);
    
    % 2. Traffic Flow Management
    % Remove distant vehicles, generate new ones, maintain 5-10 vehicles in [-50, 150] range
    raw_obs_list = update_traffic_manager(raw_obs_list, x_curr, scenario);
    
    % --- Step B: Perception and Prediction ---
    % Update obstacle prediction (generate N-step future trajectory for planner)
    current_obstacles = generate_obstacles(raw_obs_list, scenario.ego_size, dt, planner_params.N);
    
    % --- Step C: Planning ---
    [u_cmd, plan_info] = planner_step(x_curr, current_obstacles, scenario, last_plan_info, planner_params);
    last_plan_info = plan_info;
    
    % --- Step D: Execute Control ---
    x_next = update_state(x_curr, u_cmd, dt, planner_params.L);
    
    % --- Step E: Data Logging ---
    sim_log(k).time = sim_time;
    sim_log(k).ego_state = x_curr;
    sim_log(k).u_cmd = u_cmd;
    sim_log(k).obstacles = current_obstacles;
    sim_log(k).plan_info = plan_info;
    
    % Print status
    if mod(k, 10) == 0
        fprintf('Step %d/%d (%.1fs): X=%.1f, V=%.1f, Obs=%d, Action=%s, Lane_id=%d\n', ...
            k, N_steps, sim_time, x_curr(1), x_curr(4), length(raw_obs_list), plan_info.best_cand_name, ...
             plan_info.best_target_lane_id);
    end
    
    waitbar(k/N_steps, h_wait, sprintf('Time: %.1fs, Obs: %d', sim_time, length(raw_obs_list)));
    
    % Update state pointer
    x_curr = x_next;
end

close(h_wait);
fprintf('Simulation Finished.\n');

% ==========================================
% 4. Save Results
% ==========================================
save('sim_results.mat', 'sim_log', 'scenario', 'constraints_init', 'dt');
fprintf('Results saved to sim_results.mat\n');

animate_history(false);