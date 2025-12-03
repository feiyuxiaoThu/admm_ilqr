function total_cost = calculate_total_cost_admm(X, U, x_ref_traj, weights, admm_data)
N = size(U, 2);
total_cost = 0;
for i = 1:N
    step_admm = struct('sigma', admm_data.sigma);
    if isfield(admm_data, 'z_u')
        step_admm.z_u = admm_data.z_u(:, i);
        step_admm.lambda_u = admm_data.lambda_u(:, i);
    end
    if isfield(admm_data, 'z_x')
        step_admm.z_x = admm_data.z_x(:, i);
        step_admm.lambda_x = admm_data.lambda_x(:, i);
    end

    if i == 1
        u_prev = zeros(2, 1);
    else
        u_prev = U(:, i-1);
    end

    [c, ~, ~, ~, ~, ~] = get_cost_and_derivatives(X(:, i), U(:, i), u_prev, x_ref_traj(:, i), weights, step_admm, false);
    total_cost = total_cost + c;
end

% terminal cost
step_admm_end = struct('sigma', admm_data.sigma);
if isfield(admm_data, 'z_x')
    step_admm_end.z_x = admm_data.z_x(:, N+1);
    step_admm_end.lambda_x = admm_data.lambda_x(:, N+1);
end
[c, ~, ~, ~, ~, ~] = get_cost_and_derivatives(X(:, N+1), zeros(2,1), zeros(2,1), x_ref_traj(:, N+1), weights, step_admm_end, true);
total_cost = total_cost + c;
end