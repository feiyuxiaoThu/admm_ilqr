function [X_new, U_new, cost_new, cost_old] = forward_pass_admm(X_old, U_old, k_list, K_list, x_ref_traj, weights, dt, L, admm_data)
N = size(U_old, 2);
nx = 4;
nu = 2;

% Line search parameters
alpha = 1.0;
min_alpha = 1e-1;

% compute cost of the old trajectory
cost_old = calculate_total_cost_admm(X_old, U_old, x_ref_traj, weights, admm_data);

while alpha > min_alpha
    X_new = zeros(nx, N+1);
    U_new = zeros(nu, N);
    X_new(:, 1) = X_old(:, 1); % keep initial state

    % Simulate new trajectory
    for i = 1 : N
        % Compute control law (eq.17)
        delta_x = X_new(:, i) - X_old(:, i);
        delta_u = alpha * k_list(:, i) + K_list(:, :, i) * delta_x;
        U_new(:, i) = U_old(:, i) + delta_u;

        % Apply control and propagate state
        X_new(:, i+1) = update_state(X_new(:, i), U_new(:, i), dt, L); 
    end

    cost_new = calculate_total_cost_admm(X_new, U_new, x_ref_traj, weights, admm_data);

    % Accept new trajectory if cost decreased (allow 5% degradation to escape local minima)
    if cost_new < cost_old * 1.05
        return;
    else
        % Otherwise reduce alpha and retry
        alpha = alpha * 0.5;
    end
end

% If line search fails, rescue measure: accept if cost increase is within numerical noise
if cost_new - cost_old < 1e-3
    % Slightly worse trajectory accepted to avoid stagnation
    return;
else
    % Failed: rollback to old trajectory
    X_new = X_old;
    U_new = U_old;
    cost_new = cost_old;
    % Line search failed; typically indicates convergence
end
end