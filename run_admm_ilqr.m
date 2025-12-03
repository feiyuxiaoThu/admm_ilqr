function [X_opt, U_opt, debug_info] = run_admm_ilqr(x0, candidate, constraints, dt, L, weights, options, warm_start_U)
% RUN_ADMM_ILQR Execute ADMM-based constrained iLQR algorithm (Paper Algorithm 1)
%
% Inputs:
%   x0:             Initial state
%   candidate:      Reference trajectory candidate
%   constraints:    Constraint struct (.u_min, .u_max, .obstacles)
%   dt, L, weights: System parameters
%   options:        Algorithm parameters (.max_admm_iter, .max_ilqr_iter, .sigma, .tol_admm)
%   warm_start_U:   (2 x N) control sequence from previous frame (optional)
%
% Outputs:
%   X_opt, U_opt: Optimized trajectory
%   debug_info:   Debug info including convergence history
% ==========================================
% 1. Parameter Initialization
% ==========================================
if nargin < 7, options = struct(); end
if ~isfield(options, 'max_admm_iter'), options.max_admm_iter = 20; end
if ~isfield(options, 'max_ilqr_iter'), options.max_ilqr_iter = 100; end
if ~isfield(options, 'sigma'), options.sigma = 10.0; end
if ~isfield(options, 'tol_admm'), options.tol_admm = 1e-3; end

if nargin < 8, warm_start_U = []; end % Default no warm start

x_ref_traj = candidate.x_ref;
acc_ref = candidate.acc;

N = size(x_ref_traj, 2) - 1;
sigma = options.sigma;

% Initialize trajectory (Algorithm 1, Line 1)
if ~isempty(warm_start_U)
    % --- Warm Start ---
    % Use input control sequence and integrate from current x0 to get initial trajectory X
    % This ensures X_init satisfies dynamics constraint starting from current x0
    U_init = warm_start_U;
    
    % Basic integrity check: length must match
    if size(U_init, 2) ~= N
        % If length mismatch (e.g., planning horizon changed), fallback to cold start
        warning('Warm start U length mismatch. Reverting to cold start.');
        [X_init, U_init] = generate_initial_trajectory(x0, x_ref_traj, N, dt, L, constraints, acc_ref);
    else
        % Forward integration to generate corresponding X_init
        X_init = zeros(4, N+1);
        X_init(:, 1) = x0;
        for k = 1:N
            X_init(:, k+1) = update_state(X_init(:, k), U_init(:, k), dt, L);
        end
        % fprintf('  [Info] Using Warm Start.\n');
    end
else
    % --- Cold Start ---
    % Use PID/PurePursuit to generate initial values
    [X_init, U_init] = generate_initial_trajectory(x0, x_ref_traj, N, dt, L, constraints, acc_ref);
end
X = X_init;
U = U_init;

% Initialize ADMM variables
% z_u, lambda_u: projection and multiplier for control (u)
% z_x, lambda_x: projection and multiplier for state (first 2 dims of x, position)
z_u = U;
lambda_u = zeros(size(U));

z_x = X(1:2, :);
lambda_x = zeros(size(z_x));

% Record history for plotting
debug_info.cost_history = [];
debug_info.residual_history = [];
debug_info.X_history = {};
debug_info.U_history = {};

target_dot_sign = zeros(1, length(constraints.obstacles));

% fprintf('Starting ADMM-iLQR...\n');

% ==========================================
% 2. ADMM Main Loop (Algorithm 1, Line 3)
% ==========================================
for iter_admm = 1:options.max_admm_iter

    % --- Step 1: iLQR Iteration (Algorithm 1, Line 4-11) ---
    % Construct ADMM data passed to cost function
    admm_data.sigma = sigma;
    admm_data.z_u = z_u;
    admm_data.lambda_u = lambda_u;
    admm_data.z_x = z_x;
    admm_data.lambda_x = lambda_x;

    % Run iLQR
    % tic;
    [X, U, ilqr_cost, X_hist_ilqr, U_hist_ilqr] = run_iLQR_admm_wrapper(X, U, x_ref_traj, dt, L, weights, options, admm_data);
    % ilqr_time = toc;
    % fprintf('  iLQR Time: %.4f s\n', ilqr_time);

    % --- Step 2: Projection (Algorithm 1, Line 12-13) ---
    % Compute projection z = Proj(y + lambda/sigma)
    % tic;
    [z_u_new, z_x_new, target_dot_sign] = project_constraints_stable_ref(U, X, X_init, lambda_u, lambda_x, sigma, constraints, target_dot_sign, iter_admm);
    % projection_time = toc;
    % fprintf('  Projection Time: %.4f s\n', projection_time);

    % --- Step 3: Multiplier Update (Algorithm 1, Line 14-15) ---
    % lambda = lambda + sigma * (y - z)

    % Compute residual (Primal Residual)
    res_u = U - z_u_new;
    res_x = X(1:2, :) - z_x_new;

    % % Update Lambda
    % lambda_u = lambda_u + sigma * res_u;
    % lambda_x = lambda_x + sigma * res_x;

    % Update Lambda with over-relaxation
    alpha_or = options.alpha_or;
    lambda_u = lambda_u + sigma * (alpha_or * U + (1 - alpha_or) * z_u - z_u_new);
    lambda_x = lambda_x + sigma * (alpha_or * X(1:2, :) + (1 - alpha_or) * z_x - z_x_new);


    % Compute Dual Residual (dual residual), s = sigma * (z_new - z_old)
    dual_res_u = sigma * (z_u_new - z_u);
    dual_res_x = sigma * (z_x_new - z_x);
    norm_primal = norm([res_u(:); res_x(:)]);
    norm_dual = norm([dual_res_u(:); dual_res_x(:)]);

    % Update z for next iLQR
    z_u = z_u_new;
    z_x = z_x_new;

    % Adaptive sigma adjustment
    % Threshold parameters mu=10, tau=2 (classic settings)
    if options.adjust_sigma
        mu = 10;
        tau = 2;
        if norm_primal > mu * norm_dual
            sigma = sigma * tau;
            % Does lambda need to be divided by 2? Standard ADMM doesn't, but some variants rescale.
            % Here keep lambda unchanged, or to maintain lambda/sigma ratio:
            % lambda_u = lambda_u / tau;
            % lambda_x = lambda_x / tau; % (Optional, usually converges without rescaling)
            fprintf('  [Adapt] Increasing sigma to %.2f\n', sigma);
        elseif norm_dual > mu * norm_primal
            sigma = sigma / tau;
            fprintf('  [Adapt] Decreasing sigma to %.2f\n', sigma);
        end
    end

    % --- Convergence Check ---
    % Simple residual norm check
    total_res = norm(res_u(:)) + norm(res_x(:));
    debug_info.residual_history(end+1) = total_res;
    debug_info.cost_history(end+1) = ilqr_cost;
    debug_info.X_history{end+1} = X_hist_ilqr;
    debug_info.U_history{end+1} = U_hist_ilqr;

    % fprintf('ADMM Iter %2d: Cost = %.4e, Residual = %.4e\n', iter_admm, ilqr_cost, total_res);

    if total_res < options.tol_admm
        % fprintf('ADMM Converged!\n');
        break;
    end
end

X_opt = X;
U_opt = U;
end