function [X, U, final_cost, X_hist_ilqr, U_hist_ilqr] = run_iLQR_admm_wrapper(X_init, U_init, x_ref, dt, L, weights, options, admm_data)
X = X_init;
U = U_init;

X_hist_ilqr = zeros(size(X,1), size(X,2), options.max_ilqr_iter);
U_hist_ilqr = zeros(size(U,1), size(U,2), options.max_ilqr_iter);
actual_iters = 0;

for iter = 1:options.max_ilqr_iter
    % Backward Pass (pass admm_data)
    % Note: backward_pass function interface needs corresponding update
    % tic;
    [k_list, K_list, success] = backward_pass_admm(X, U, x_ref, weights, dt, L, admm_data);
    % backward_time = toc;
    % fprintf('  Backward Pass Time: %.4f s\n', backward_time);

    if ~success
        % If failed, usually means regularization couldn't help, exit current iLQR
        break;
    end

    % tic;
    % Forward Pass (pass admm_data for line search cost computation)
    [X_new, U_new, cost_new, cost_old] = forward_pass_admm(X, U, k_list, K_list, x_ref, weights, dt, L, admm_data);
    % forward_time = toc;
    % fprintf('  Forward Pass Time: %.4f s\n', forward_time);

    % Simple cost change check
    if iter == 1
        current_cost = cost_old;
    end

    cost_change = abs(current_cost - cost_new);
    u_change = norm(U_new - U);

    X = X_new;
    U = U_new;
    current_cost = cost_new;

    actual_iters = actual_iters + 1;
    X_hist_ilqr(:, :, actual_iters) = X;
    U_hist_ilqr(:, :, actual_iters) = U;

    if cost_change < options.ilqr_tol && u_change < options.ilqr_tol
        % fprintf('iLQR converged at iteration %d: Cost = %.4f, U_change = %.4f\n', iter, cost_new, u_change);
        X_hist_ilqr = X_hist_ilqr(:, :, 1:actual_iters);
        U_hist_ilqr = U_hist_ilqr(:, :, 1:actual_iters);
        break;
    end
end
final_cost = current_cost;
end
