function [k_list, K_list, success] = backward_pass_admm(X, U, x_ref_traj, weights, dt, L, admm_data)
N = size(U, 2);
nx = 4;
nu = 2;

k_list = zeros(nu, N);
K_list = zeros(nu, nx, N);

% 1. Terminal cost
% Assume terminal cost has same form as running cost or is defined separately
step_admm = struct('sigma', admm_data.sigma);
if isfield(admm_data, 'z_x'), step_admm.z_x = admm_data.z_x(:, N+1); end
if isfield(admm_data, 'lambda_x'), step_admm.lambda_x = admm_data.lambda_x(:, N+1); end
[~, Vx, ~, Vxx, ~, ~] = get_cost_and_derivatives(X(:, N+1), zeros(nu,1), zeros(nu,1), x_ref_traj(:, N+1), weights, step_admm, true);
success = true;

% Regularization parameters initialization
% Initial regularization parameters for numerical stability
reg_mu = 1e-6;      % initial regularization
reg_min = 1e-6;     % minimum regularization
reg_max = 1e3;      % maximum regularization (failure if exceeded)
reg_scale = 10;     % multiplier for increasing reg_mu

% 2. Backward pass
for i = N:-1:1
    x = X(:, i);
    u = U(:, i);
    x_ref = x_ref_traj(:, i);

    if i == 1
        u_prev = zeros(2, 1);
    else
        u_prev = U(:, i-1);
    end

    step_admm = struct('sigma', admm_data.sigma);
    % Extract control constraint data for step i
    if isfield(admm_data, 'z_u')
        step_admm.z_u = admm_data.z_u(:, i);
        step_admm.lambda_u = admm_data.lambda_u(:, i);
    end
    % Extract state constraint data for step i
    if isfield(admm_data, 'z_x')
        step_admm.z_x = admm_data.z_x(:, i);
        step_admm.lambda_x = admm_data.lambda_x(:, i);
    end

    % dynamics Jacobians
    A = get_A_matrix(x, u, dt, L);
    B = get_B_matrix(x, u, dt, L);

    % cost derivatives
    [~, lx, lu, lxx, luu, lux] = get_cost_and_derivatives(x, u, u_prev, x_ref, weights, step_admm, false);

    % compute Q-function derivatives (eq.14)
    Qx = lx + A' * Vx;
    Qu = lu + B' * Vx;
    Qxx = lxx + A' * Vxx * A;
    Qux = lux + B' * Vxx * A;
    Quu = luu + B' * Vxx * B;

    % regularize Quu for numerical stability
    % find mu such that (Quu + mu*I) is positive definite
    is_spd = false;    % flag: symmetric positive definite
    Quu_reg = Quu;     % initialize

    % Try increasing regularization until PD or reach limit
    while ~is_spd
        Quu_reg = Quu + reg_mu * eye(nu);

        % Check positive-definiteness via Cholesky: p==0 means PD
        [R_chol, p_chol] = chol(Quu_reg);

        if p_chol == 0
            % success: matrix is PD
            is_spd = true;
        else
            fprintf('Regularizing Quu at step %d with mu = %.2e\n', i, reg_mu);
            % increase regularization
            reg_mu = reg_mu * reg_scale;

            % abort if exceeded maximum allowed regularization
            if reg_mu > reg_max
                success = false;
                warning('Backward pass failed at step %d: Quu regularization exceeded max limit.', i);
                return;
            end
        end
    end

    % k = -Quu_reg \ Qu;
    % K = -Quu_reg \ Qux;

    % compute control gains (use Cholesky factor R for numerical stability)
    % Quu_reg * k = -Qu  =>  R'*R*k = -Qu  => k = -R\(R'\Qu)
    k = -R_chol \ (R_chol' \ Qu);
    K = -R_chol \ (R_chol' \ Qux);

    k_list(:, i) = k;
    K_list(:, :, i) = K;

    % update V derivatives (eq.18)
    Vx = Qx - K' * Quu_reg * k;
    Vxx = Qxx - K' * Quu_reg * K;

    % enforce symmetry (numeric errors may break symmetry)
    Vxx = 0.5 * (Vxx + Vxx');

    % If regularization succeeded at this step, try reducing reg_mu for next step
    % to stay numerically safe while approaching the unregularized solution
    reg_mu = max(reg_min, reg_mu / reg_scale);
end
end