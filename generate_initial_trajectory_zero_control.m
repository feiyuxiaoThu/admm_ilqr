function [X, U] = generate_initial_trajectory_zero_control(x0, x_ref_traj, N, dt, L, constraints)
    % Generate initial nominal trajectory (zero controls)
    nx = 4; nu = 2;
    U = zeros(nu, N);        % N control steps
    X = zeros(nx, N+1);      % N+1 states (including initial)
    
    X(:, 1) = x0;
    for k = 1:N
        X(:, k+1) = update_state(X(:, k), U(:, k), dt, L);
    end
end