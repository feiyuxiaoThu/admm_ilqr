function [X_opt, U_opt, debug_info] = run_admm_ilqr(x0, candidate, constraints, dt, L, weights, options)
% RUN_ADMM_ILQR 执行基于ADMM的约束iLQR算法 (论文 Algorithm 1)
%
% 输入:
%   x0: 初始状态
%   x_ref_traj: 参考轨迹
%   constraints: 约束结构体 (.u_min, .u_max, .obstacles)
%   dt, L, weights: 系统参数
%   options: 算法参数 (.max_admm_iter, .max_ilqr_iter, .sigma, .tol_admm)
%
% 输出:
%   X_opt, U_opt: 优化后的轨迹
%   debug_info: 包含收敛历史等调试信息
% ==========================================
% 1. 参数初始化
% ==========================================
if nargin < 7, options = struct(); end
if ~isfield(options, 'max_admm_iter'), options.max_admm_iter = 20; end
if ~isfield(options, 'max_ilqr_iter'), options.max_ilqr_iter = 100; end
if ~isfield(options, 'sigma'), options.sigma = 10.0; end
if ~isfield(options, 'tol_admm'), options.tol_admm = 1e-3; end

x_ref_traj = candidate.x_ref;
acc_ref = candidate.acc;

N = size(x_ref_traj, 2) - 1;
sigma = options.sigma;

% 初始化轨迹 (Algorithm 1, Line 1)
[X_init, U_init] = generate_initial_trajectory(x0, x_ref_traj, N, dt, L, constraints, acc_ref);
X = X_init;
U = U_init;

% 初始化 ADMM 变量
% z_u, lambda_u: 控制量的投影和乘子 (对应 u)
% z_x, lambda_x: 状态量的投影和乘子 (对应 x 的前两维位置)
z_u = U;
lambda_u = zeros(size(U));

z_x = X(1:2, :);
lambda_x = zeros(size(z_x));

% 记录历史用于绘图
debug_info.cost_history = [];
debug_info.residual_history = [];
debug_info.X_history = {};
debug_info.U_history = {};

target_dot_sign = zeros(1, length(constraints.obstacles));

fprintf('Starting ADMM-iLQR...\n');

% ==========================================
% 2. ADMM 主循环 (Algorithm 1, Line 3)
% ==========================================
for iter_admm = 1:options.max_admm_iter

    % --- Step 1: iLQR Iteration (Algorithm 1, Line 4-11) ---
    % 构造传递给 cost 函数的 ADMM 数据
    admm_data.sigma = sigma;
    admm_data.z_u = z_u;
    admm_data.lambda_u = lambda_u;
    admm_data.z_x = z_x;
    admm_data.lambda_x = lambda_x;

    % 运行 iLQR
    % tic;
    [X, U, ilqr_cost, X_hist_ilqr, U_hist_ilqr] = run_iLQR_admm_wrapper(X, U, x_ref_traj, dt, L, weights, options, admm_data);
    % ilqr_time = toc;
    % fprintf('  iLQR Time: %.4f s\n', ilqr_time);

    % --- Step 2: Projection (Algorithm 1, Line 12-13) ---
    % 计算投影 z = Proj(y + lambda/sigma)
    % tic;
    [z_u_new, z_x_new, target_dot_sign] = project_constraints_stable_ref(U, X, X_init, lambda_u, lambda_x, sigma, constraints, target_dot_sign, iter_admm);
    % projection_time = toc;
    % fprintf('  Projection Time: %.4f s\n', projection_time);

    % --- Step 3: Multiplier Update (Algorithm 1, Line 14-15) ---
    % lambda = lambda + sigma * (y - z)

    % 计算残差 (Primal Residual)
    res_u = U - z_u_new;
    res_x = X(1:2, :) - z_x_new;

    % % 更新 Lambda
    % lambda_u = lambda_u + sigma * res_u;
    % lambda_x = lambda_x + sigma * res_x;

    % 更新 Lambda with over relaxation
    alpha_or = options.alpha_or;
    lambda_u = lambda_u + sigma * (alpha_or * U + (1 - alpha_or) * z_u - z_u_new);
    lambda_x = lambda_x + sigma * (alpha_or * X(1:2, :) + (1 - alpha_or) * z_x - z_x_new);


    % 计算 Dual Residual (对偶残差)，s = sigma * (z_new - z_old)
    dual_res_u = sigma * (z_u_new - z_u);
    dual_res_x = sigma * (z_x_new - z_x);
    norm_primal = norm([res_u(:); res_x(:)]);
    norm_dual = norm([dual_res_u(:); dual_res_x(:)]);

    % 更新 z (供下一次 iLQR 使用)
    z_u = z_u_new;
    z_x = z_x_new;

    % 自适应调整 Sigma
    % 阈值参数 mu=10, tau=2 (经典设置)
    if options.adjust_sigma
        mu = 10;
        tau = 2;
        if norm_primal > mu * norm_dual
            sigma = sigma * tau;
            % lambda 需要除以 2 吗？标准ADMM不需要，但有些变种会 rescaling
            % 这里保持 lambda 不变，或者为了保持 lambda/sigma 比例：
            % lambda_u = lambda_u / tau;
            % lambda_x = lambda_x / tau; % (可选，通常不缩放也收敛)
            fprintf('  [Adapt] Increasing sigma to %.2f\n', sigma);
        elseif norm_dual > mu * norm_primal
            sigma = sigma / tau;
            fprintf('  [Adapt] Decreasing sigma to %.2f\n', sigma);
        end
    end

    % --- 收敛性检查 ---
    % 简单的残差范数检查
    total_res = norm(res_u(:)) + norm(res_x(:));
    debug_info.residual_history(end+1) = total_res;
    debug_info.cost_history(end+1) = ilqr_cost;
    debug_info.X_history{end+1} = X_hist_ilqr;
    debug_info.U_history{end+1} = U_hist_ilqr;

    fprintf('ADMM Iter %2d: Cost = %.4e, Residual = %.4e\n', iter_admm, ilqr_cost, total_res);

    if total_res < options.tol_admm
        fprintf('ADMM Converged!\n');
        break;
    end
end

X_opt = X;
U_opt = U;
end