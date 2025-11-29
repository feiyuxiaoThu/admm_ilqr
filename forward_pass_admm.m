function [X_new, U_new, cost_new, cost_old] = forward_pass_admm(X_old, U_old, k_list, K_list, x_ref_traj, weights, dt, L, admm_data)
N = size(U_old, 2);
nx = 4;
nu = 2;

% line search parameters
alpha = 1.0;
min_alpha = 1e-1;

% compute cost of the old trajectory
cost_old = calculate_total_cost_admm(X_old, U_old, x_ref_traj, weights, admm_data);

while alpha > min_alpha
    X_new = zeros(nx, N+1);
    U_new = zeros(nu, N);
    X_new(:, 1) = X_old(:, 1); % keep initial state

    % simulate new trajectory
    for i = 1 : N
        % compute control (eq.17)
        delta_x = X_new(:, i) - X_old(:, i);
        delta_u = alpha * k_list(:, i) + K_list(:, :, i) * delta_x;
        U_new(:, i) = U_old(:, i) + delta_u;

        % apply control and propagate state
        X_new(:, i+1) = update_state(X_new(:, i), U_new(:, i), dt, L); 
    end

    cost_new = calculate_total_cost_admm(X_new, U_new, x_ref_traj, weights, admm_data);

    % accept new trajectory if cost decreased
    if cost_new < cost_old * 1.05 % 允许 5% 的恶化以跳出局部极小
        return;
    else
        alpha = alpha * 0.5; % otherwise reduce alpha and retry
    end
end

% 如果 Line Search 失败，挽救措施：如果 alpha 缩到最小计算出的 cost_new 只比 cost_old 大一点点（数值噪声），也接受
if cost_new - cost_old < 1e-3
    % warning('Accepted slightly worse trajectory to avoid stagnation.');
    return;
else
    % 真的失败了，回滚
    X_new = X_old;
    U_new = U_old;
    cost_new = cost_old;
    % warning('Line search failed to find a better trajectory.');
    % 这种时候通常意味着收敛了，不需要 panic
end
end