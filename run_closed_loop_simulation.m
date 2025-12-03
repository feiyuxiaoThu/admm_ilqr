% run_closed_loop_simulation.m
% 闭环仿真主程序: 物理环境更新 -> 交通流管理 -> 感知 -> 规划 -> 控制 -> 记录

clear; clc; close all;

% ==========================================
% 1. 仿真配置 (Simulation Config)
% ==========================================
T_sim = 5.0;       % 总仿真时长 (s) -> 增加时长以测试无限流
dt = 0.1;           % 仿真步长 (s)
N_steps = floor(T_sim / dt);

% 规划器参数
planner_params.dt = dt;
planner_params.N = 60;  % 预测时域步数 (8.0s)
planner_params.L = 3.0; % 轴距

% 权重配置 (与 main_ADMM_iLQR.m 保持一致)
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

% 评估权重
planner_params.eval_weights.w_safety      = 2.0;
planner_params.eval_weights.w_progress    = 10.0;
planner_params.eval_weights.w_ref_vel     = 0.5;
planner_params.eval_weights.w_acc         = 0.1;
planner_params.eval_weights.w_steer       = 100.0;
planner_params.eval_weights.w_jerk_lon    = 0.1;
planner_params.eval_weights.w_jerk_lat    = 100.0;
planner_params.eval_weights.w_consistency = 500.0;

% 求解器选项
planner_params.options.max_admm_iter = 10;
planner_params.options.sigma = 10.0;
planner_params.options.tol_admm = 1e-1;
planner_params.options.alpha_or = 1.0;
planner_params.options.adjust_sigma = false;
planner_params.options.max_ilqr_iter = 50;
planner_params.options.ilqr_tol = 1e-1;

% ==========================================
% 2. 环境初始化 (Environment Init)
% ==========================================
% 构建静态场景及初始障碍物
[scenario, constraints_init, x0, raw_obs_list] = build_simulation_scenario(planner_params.dt, planner_params.N);
planner_params.constraints = constraints_init; 

% 自车状态初始化 [x; y; theta; v]
x_curr = x0;
last_plan_info = [];

% 数据日志
sim_log = struct();

fprintf('Simulation Started... Total Steps: %d\n', N_steps);
h_wait = waitbar(0, 'Simulation in progress...');

% ==========================================
% 3. 仿真主循环 (Main Loop)
% ==========================================
for k = 1:N_steps
    sim_time = (k-1) * dt;
    
    % --- Step A: 物理更新 (Physics Update) ---
    % 1. 障碍物动力学更新
    raw_obs_list = update_obstacle_physics(raw_obs_list, dt);
    
    % 2. [新增] 交通流管理 (Traffic Manager)
    % 移除远处的车，生成新车，保持 [-50, 150] 范围内有 5-10 辆车
    raw_obs_list = update_traffic_manager(raw_obs_list, x_curr, scenario);
    
    % --- Step B: 感知与预测 (Perception) ---
    % 更新障碍物预测 (生成未来 N 步轨迹传给规划器)
    current_obstacles = generate_obstacles(raw_obs_list, scenario.ego_size, dt, planner_params.N);
    
    % --- Step C: 规划 (Planning) ---
    [u_cmd, plan_info] = planner_step(x_curr, current_obstacles, scenario, last_plan_info, planner_params);
    last_plan_info = plan_info;
    
    % --- Step D: 执行控制 (Execute Control) ---
    x_next = update_state(x_curr, u_cmd, dt, planner_params.L);
    
    % --- Step E: 数据记录 (Logging) ---
    sim_log(k).time = sim_time;
    sim_log(k).ego_state = x_curr;
    sim_log(k).u_cmd = u_cmd;
    sim_log(k).obstacles = current_obstacles;
    sim_log(k).plan_info = plan_info;
    
    % 打印状态
    if mod(k, 10) == 0
        fprintf('Step %d/%d (%.1fs): X=%.1f, V=%.1f, Obs=%d, Action=%s, Lane_id=%d\n', ...
            k, N_steps, sim_time, x_curr(1), x_curr(4), length(raw_obs_list), plan_info.best_cand_name, ...
             plan_info.best_target_lane_id);
    end
    
    waitbar(k/N_steps, h_wait, sprintf('Time: %.1fs, Obs: %d', sim_time, length(raw_obs_list)));
    
    % 更新状态指针
    x_curr = x_next;
end

close(h_wait);
fprintf('Simulation Finished.\n');

% ==========================================
% 4. 保存结果
% ==========================================
save('sim_results.mat', 'sim_log', 'scenario', 'constraints_init', 'dt');
fprintf('Results saved to sim_results.mat\n');

% 简单绘图
% figure;
% subplot(2,1,1);
% v_hist = arrayfun(@(s) s.ego_state(4), sim_log);
% t_hist = arrayfun(@(s) s.time, sim_log);
% plot(t_hist, v_hist, 'LineWidth', 2);
% ylabel('Velocity (m/s)'); grid on; title('Ego Velocity');
% subplot(2,1,2);
% num_obs_hist = arrayfun(@(s) length(s.obstacles), sim_log);
% plot(t_hist, num_obs_hist, 'LineWidth', 2);
% ylabel('Num Obstacles'); xlabel('Time (s)'); grid on; title('Visible Obstacles Count');

animate_history(false);