% clear environment
clc; clear; 
close all;

% 1. Global parameters
dt = 0.1;           % time step (s)
L = 3.0;            % wheelbase (m)
v_target = 10.0;    % target speed (m/s)
total_time = 8.0;   % total simulation time (s)
N = floor(total_time / dt);

% Weights
weights.q_pos_x = 0.0;    % position x penalty
weights.q_pos_y = 0.2;    % position y penalty
weights.q_vel = 1.0;    % velocity penalty
weights.r_acc = 0.5;    % acceleration penalty
weights.r_steer = 50.0; % steering penalty
weights.q_pos_x_term = 0.0; % X position terminal penalty
weights.q_pos_y_term = 50.0; % Y position terminal penalty
weights.q_vel_term = 10.0;  % velocity terminal penalty
weights.r_delta_acc = 10.0;
weights.r_delta_steer = 10.0;

% iLQR parameters
options.max_admm_iter = 100;     % ADMM 最大迭代次数
options.sigma = 10.0;           % ADMM 惩罚参数 (初始)
options.tol_admm = 1e-1;        % ADMM 收敛容差
options.alpha_or = 1.0;        % ADMM 过松弛参数，1.0 表示无过松弛 (ADMM 收敛容差较大时（1e-1）不需要改)
options.adjust_sigma = false;   % 是否自适应调整 sigma (开启收敛更慢)
options.max_ilqr_iter = 50;     % 内部 iLQR 最大迭代次数
options.ilqr_tol = 1e-1;        % iLQR 收敛容差

% 2. 定义约束 (Constraints)
% 限制加速度 [-3, 2] m/s^2
% 限制前轮转角 [-0.5, 0.5] rad (约28度)
constraints.u_min = [-3.0; -0.5];
constraints.u_max = [ 2.0;  0.5];

% === 自车尺寸 (用于膨胀障碍物) ===
ego_size.length = 4.5;
ego_size.width  = 2.0;

% === 定义动态障碍物 (Raw Data) ===
% 格式: x, y (中心), length, width, theta (航向), vx, vy (速度)
obs_1 = struct('x', 10.0, 'y', 4.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 10.0, 'vy', 0.0);
obs_2 = struct('x', 0.0, 'y', 4.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 7.0, 'vy', 0.0); 
obs_3 = struct('x', 40.0, 'y', 0.0, 'length', 4.5, 'width', 2.0, 'theta', 0.0, 'vx', 0.0, 'vy', 0.0);
obs_list = [obs_3];
% === 生成处理后的障碍物 (包含预测轨迹和膨胀椭圆) ===
constraints.obstacles = generate_obstacles(obs_list, ego_size, dt, N);

% 3. Select test scenario, build reference trajectory
% scenario_id = 1 (straight), 2 (circle), 3 (intersection: straight - turn - straight)
scenario_id = 1;
fprintf('Testing scenario ID: %d ...\n', scenario_id);
x_ref_traj = build_reference_trajectory(scenario_id, v_target, dt, N);
x0 = [0; 0; 0; 10];

% 4. Run iLQR
tic;
[X_opt, U_opt, debug_info] = run_admm_ilqr(x0, x_ref_traj, constraints, dt, L, weights, options);
elapsed_time = toc;
fprintf('Done. Elapsed time: %.4f s\n', elapsed_time);

% 5. Plot results
% plot all stored iterations (if any)
plot_ilqr_iter = false;
fig_plot = figure('Name', 'iLQR iteration history', 'NumberTitle', 'off');
if exist('fig_plot','var')
    plot_iteration(fig_plot, dt, debug_info, constraints, x_ref_traj, plot_ilqr_iter, ego_size);
end
plot_results(X_opt, U_opt, x_ref_traj, dt, scenario_id, constraints);

