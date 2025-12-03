function [z_u, z_x, target_dot_sign] = project_constraints_stable_ref(u_traj, x_traj, init_traj, lambda_u, lambda_x, sigma, constraints, target_dot_sign, iter_admm)
% PROJECT_CONSTRAINTS_STABLE_REF (Dynamic Obstacle Support)
% 1. Stability core: use init_traj to determine projection normal direction, avoid symmetry traps.
% 2. Dynamic support: read predicted obstacle positions at each time step k.
% 3. Projection execution: ray-ellipse analytic intersection.

% ==========================================
% 1. Control constraint projection
% ==========================================
y_u = u_traj + lambda_u / sigma;
z_u = max(constraints.u_min, min(constraints.u_max, y_u));

% ==========================================
% 2. Obstacle constraint projection
% ==========================================
% ADMM intermediate variables (noisy points to be projected)
y_x = x_traj(1:2, :) + lambda_x(1:2, :) / sigma;

% Initialize output (default: no movement)
z_x = y_x;

if ~isfield(constraints, 'obstacles') || isempty(constraints.obstacles)
    return;
end

num_steps = size(y_x, 2);
num_obs = length(constraints.obstacles);

% --- [Pre-calc]: Compute stable normal field based on reference trajectory ---
% This part depends only on reference line, computed once
ref_pos = init_traj(1:2, 1:num_steps);
dx_ref = gradient(ref_pos(1, :));
dy_ref = gradient(ref_pos(2, :));
norm_ref = sqrt(dx_ref.^2 + dy_ref.^2);
norm_ref(norm_ref < 1e-6) = 1;

% Tangent vector
tx_ref = dx_ref ./ norm_ref;
ty_ref = dy_ref ./ norm_ref;

% Left normal vector: [-ty, tx]
nx_ref = -ty_ref;
ny_ref = tx_ref;

% Cumulative displacement field (superposition of all obstacle pushes)
total_shift = zeros(2, num_steps);

% ==========================================
% Outer loop: iterate over obstacles
% ==========================================
for j = 1:num_obs
    obs = constraints.obstacles(j);
    a = obs.a;
    b = obs.b;

    % Check if dynamic prediction trajectory exists
    % Assume structure: obs.prediction.trajectory (3 x M) -> [x; y; theta]
    is_dynamic = isfield(obs, 'prediction') && ~isempty(obs.prediction);

    % ==========================================
    % Inner loop: iterate over time steps (point-by-point processing, non-vectorized)
    % ==========================================
    for k = 1:num_steps

        % --- 1. Get obstacle pose at current time step k ---
        if is_dynamic
            pred_traj = obs.prediction;
            pred_len = size(pred_traj, 2);
            % Prevent index out of bounds (if prediction too short, keep last frame)
            idx = min(k, pred_len);

            curr_x = pred_traj(1, idx);
            curr_y = pred_traj(2, idx);
            curr_theta = pred_traj(3, idx);
        else
            % Static obstacle
            curr_x = obs.x;
            curr_y = obs.y;
            curr_theta = obs.theta;
        end

        center = [curr_x; curr_y];

        % Compute rotation matrix at current time (Local -> Global)
        cos_t = cos(curr_theta);
        sin_t = sin(curr_theta);
        R = [cos_t, -sin_t; sin_t, cos_t];

        % Compute collision detection matrix at current time
        Sigma_inv = diag([1/a^2, 1/b^2]);
        A_mat = R * Sigma_inv * R';

        % --- 2. Collision detection ---
        pt_global = y_x(:, k);
        d_vec = pt_global - center;

        % Ellipse equation check: d' * A * d
        metric = d_vec' * A_mat * d_vec;

        % If no collision, skip current point
        if metric >= 1.0
            continue;
        end

        % --- 3. [Topology decision] Determine which side of reference trajectory the obstacle is on ---
        % Note: use reference point at time k and obstacle center at time k
        if target_dot_sign(j) == 0
            ref_pt_k = ref_pos(:, k);
            vec_to_center = center - ref_pt_k;

            % Get tangent vector at current reference point
            tx_k = tx_ref(k);
            ty_k = ty_ref(k);

            % 2D cross product: Ref_Tangent x Vector_To_Obs
            % > 0: obstacle on left -> vehicle dodge right
            % < 0: obstacle on right -> vehicle dodge left
            cross_val = tx_k * vec_to_center(2) - ty_k * vec_to_center(1);

            if cross_val > 0
                target_dot_sign(j) = -1.0; % Target: dodge right (negative normal)
            else
                target_dot_sign(j) = 1.0;  % Target: dodge left (positive normal)
            end
        end


        % --- 4. Prepare projection ray ---
        % Use reference trajectory normal as projection direction (ensures stability)
        n_curr = [nx_ref(k); ny_ref(k)];

        % Singular value protection
        if norm(n_curr) < 1e-3
            n_curr = [-sin_t; cos_t];
        end

        %% Debug: plotting routine (strictly preserve your code structure)
        % figure('Position', [100, 100, 1200, 600]); hold on; axis equal; 
        % plot(pt_global(1), pt_global(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Colliding Point');
        % % Define unit circle for transformation
        % theta_circle = linspace(0, 2*pi, 100);
        % circle_x = cos(theta_circle);
        % circle_y = sin(theta_circle);
        % % --- Ellipse transformation logic ---
        % % 1. Scale
        % scale_x = obs.a * circle_x;
        % scale_y = obs.b * circle_y;
        % % 2. Rotate (use curr_theta at current time)
        % R_draw = [cos(curr_theta), -sin(curr_theta);
        %     sin(curr_theta),  cos(curr_theta)];
        % rotated_points = R_draw * [scale_x; scale_y];
        % % 3. Translate (use curr_x, curr_y at current time)
        % final_x = rotated_points(1, :) + curr_x;
        % final_y = rotated_points(2, :) + curr_y;
        % % 4. Plot
        % plot(final_x, final_y, 'r-', 'LineWidth', 1, 'DisplayName', 'Obstacle');
        % % Plot center point
        % plot(curr_x, curr_y, 'r+',  'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
        % plot(x_traj(1, :), x_traj(2, :), 'b-', 'DisplayName', 'Trajectory');
        % plot(y_x(1, :), y_x(2, :), 'g--', 'DisplayName', 'y\_x Points');
        % plot(init_traj(1, :), init_traj(2, :), 'k:', 'DisplayName', 'Reference Traj');
        % xlim([curr_x - obs.a*2, curr_x + obs.a*2]);
        % ylim([curr_y - obs.b*2, curr_y + obs.b*2]);
        %% End debug plotting

        % --- 5. Ray-ellipse analytic intersection ---
        % Ray equation: P(t) = P_local + t * Dir_local

        % Convert global ray to obstacle local coordinate frame
        % P_local: colliding point in local coordinates relative to center
        pt_local = R' * (pt_global - center);
        u0 = pt_local(1);
        v0 = pt_local(2);

        % Dir_local: reference normal in local coordinate frame
        dir_local = R' * n_curr;
        du = dir_local(1);
        dv = dir_local(2);

        % Solve quadratic: A*t^2 + B*t + C = 0
        % Substitute into ellipse equation (u0 + t*du)^2/a^2 + (v0 + t*dv)^2/b^2 = 1
        Coeff_A = (du/a)^2 + (dv/b)^2;
        Coeff_B = 2 * ( (u0*du)/a^2 + (v0*dv)/b^2 );
        Coeff_C = (u0/a)^2 + (v0/b)^2 - 1;

        delta = Coeff_B^2 - 4 * Coeff_A * Coeff_C;

        if delta < 0
            % Theoretically if point inside ellipse, solution must exist. No solution likely due to numerical error, skip
            continue;
        end

        sqrt_delta = sqrt(delta);
        t1 = (-Coeff_B + sqrt_delta) / (2 * Coeff_A);
        t2 = (-Coeff_B - sqrt_delta) / (2 * Coeff_A);

        % Compute two candidate displacement vectors (Global Frame)
        % t1, t2 are distances along n_curr
        shift1 = t1 * n_curr;
        shift2 = t2 * n_curr;

        % --- 6. Target selection ---
        % Choose correct t based on target_dot_sign
        % target > 0 (Left): choose larger t (positive direction)
        % target < 0 (Right): choose smaller t (negative direction)

        if target_dot_sign(j) > 0
            if t1 > t2, shift_vec = shift1; else, shift_vec = shift2; end
        else
            if t1 < t2, shift_vec = shift1; else, shift_vec = shift2; end
        end

        % Accumulate force (Shift)
        total_shift(:, k) = total_shift(:, k) + shift_vec;

        %% Debug: plot after collision projection
        % Q_global = y_x(:, k) + shift_vec;
        % plot(Q_global(1), Q_global(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Projected Point');
        % legend('Location', 'best');
        % title(sprintf('Obstacle %d at Time Step %d, ADMM iter %d', j, k, iter_admm));
        % close all;
        %% End debug plotting

    end % end k loop
end % end obs loop

% 3. Apply resultant force and execute road boundary clamping (Strategy B)

% First apply collision avoidance push
z_x_temp = y_x + total_shift;

% Then execute hard clamping
if isfield(constraints, 'road_bounds') && ~isempty(constraints.road_bounds)
    y_min = constraints.road_bounds(1);
    y_max = constraints.road_bounds(2);
    
    % Clamp Y-axis (row 2)
    % Logic: max(y_min, min(y_max, val))
    z_x_temp(2, :) = max(y_min, min(y_max, z_x_temp(2, :)));
end

z_x = z_x_temp;

end
