function [z_u, z_x, target_dot_sign] = project_constraints_stable_ref_barrier_enhanced(u_traj, x_traj, init_traj, lambda_u, lambda_x, sigma, constraints, target_dot_sign)
% PROJECT_CONSTRAINTS_STABLE_REF (Dynamic Obstacle Support with DCBF)
% 1. Stability core: use init_traj to determine projection normal direction.
% 2. Dynamic support: read predicted obstacle positions at each time step k.
% 3. Projection execution: ray-ellipse analytic intersection.
% 4. Safety: Discrete-time Control Barrier Function (DCBF) for safety recovery.

% ==========================================
% 1. Control constraint projection
% ==========================================
y_u = u_traj + lambda_u / sigma;
z_u = max(constraints.u_min, min(constraints.u_max, y_u));

% ==========================================
% 2. Obstacle constraint projection (with DCBF)
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

% --- [DCBF Configuration] ---
% Generate varying barrier coefficient alpha_k from 0.2 to 1.0
% This allows safety recovery: smaller alpha at beginning allows gradual return to safety.
alpha_start = 0.2;
alpha_end = 1.0;
alpha_seq = linspace(alpha_start, alpha_end, num_steps);

% ==========================================
% Outer loop: iterate over obstacles
% ==========================================
for j = 1:num_obs
    obs = constraints.obstacles(j);
    a_base = obs.a; % Original semi-major axis
    b_base = obs.b; % Original semi-minor axis

    % Check if dynamic prediction trajectory exists
    is_dynamic = isfield(obs, 'prediction') && ~isempty(obs.prediction);
    
    % Initialize h_prev for DCBF (will be set at k=1)
    h_prev = 0;

    % ==========================================
    % Inner loop: iterate over time steps
    % ==========================================
    for k = 1:num_steps

        % --- 1. Get obstacle pose at current time step k ---
        if is_dynamic
            pred_traj = obs.prediction;
            pred_len = size(pred_traj, 2);
            idx = min(k, pred_len);
            curr_x = pred_traj(1, idx);
            curr_y = pred_traj(2, idx);
            curr_theta = pred_traj(3, idx);
        else
            curr_x = obs.x;
            curr_y = obs.y;
            curr_theta = obs.theta;
        end

        center = [curr_x; curr_y];

        % Compute rotation matrix at current time (Local -> Global)
        cos_t = cos(curr_theta);
        sin_t = sin(curr_theta);
        R = [cos_t, -sin_t; sin_t, cos_t];

        % Compute collision detection matrix using ORIGINAL axes
        Sigma_inv = diag([1/a_base^2, 1/b_base^2]);
        A_mat = R * Sigma_inv * R';

        % --- 2. DCBF Logic Check ---
        pt_global = y_x(:, k);
        d_vec = pt_global - center;

        % Ellipse metric: d' * A * d. 
        % metric < 1.0 means inside (unsafe), metric >= 1.0 means outside (safe).
        metric = d_vec' * A_mat * d_vec;
        
        % Define Barrier Function h: h(x) = sqrt(metric) - 1
        % h >= 0 (Safe), h < 0 (Unsafe)
        h_curr = sqrt(metric) - 1;

        % Special handling for k=1 (Initial State)
        if k == 1
            % We generally assume k=1 is the current physical state and cannot be projected instantly.
            % We just initialize h_prev based on current state.
            h_prev = h_curr;
            continue; 
        end
        
        % Calculate Minimum Allowed h based on DCBF rule:
        % h(k) >= (1 - alpha_k) * h(k-1)
        % This creates a dynamic safety boundary.
        alpha_k = alpha_seq(k);
        h_min_allowed = (1 - alpha_k) * h_prev;
        
        % --- Check if projection is needed ---
        if h_curr >= h_min_allowed
            % Satisfies DCBF constraint (either safe, or recovering fast enough)
            % Update h_prev using actual current value
            h_prev = h_curr;
            continue; 
        end
        
        % --- Projection Needed ---
        % We need to project the point such that its new h equals h_min_allowed.
        % New target metric = (1 + h_min_allowed)^2
        % This is equivalent to projecting onto a scaled ellipse.
        
        target_scale = 1.0 + h_min_allowed;
        
        % Safety clamp to prevent negative or zero scale (numerical issues)
        if target_scale < 0.01
            target_scale = 0.01;
        end
        
        % Scale axes for the "Virtual Ellipse" target
        a_target = a_base * target_scale;
        b_target = b_base * target_scale;

        % --- 3. [Topology decision] ---
        if target_dot_sign(j) == 0
            ref_pt_k = ref_pos(:, k);
            vec_to_center = center - ref_pt_k;
            tx_k = tx_ref(k);
            ty_k = ty_ref(k);
            cross_val = tx_k * vec_to_center(2) - ty_k * vec_to_center(1);

            if cross_val > 0
                target_dot_sign(j) = -1.0; % Target: dodge right
            else
                target_dot_sign(j) = 1.0;  % Target: dodge left
            end
        end

        % --- 4. Prepare projection ray ---
        n_curr = [nx_ref(k); ny_ref(k)];
        if norm(n_curr) < 1e-3
            n_curr = [-sin_t; cos_t];
        end

        % --- 5. Ray-Ellipse analytic intersection (with SCALED axes) ---
        % Convert global ray to obstacle local frame
        pt_local = R' * (pt_global - center);
        u0 = pt_local(1);
        v0 = pt_local(2);

        dir_local = R' * n_curr;
        du = dir_local(1);
        dv = dir_local(2);

        % Solve intersection with Scaled Ellipse (a_target, b_target)
        Coeff_A = (du/a_target)^2 + (dv/b_target)^2;
        Coeff_B = 2 * ( (u0*du)/a_target^2 + (v0*dv)/b_target^2 );
        Coeff_C = (u0/a_target)^2 + (v0/b_target)^2 - 1;

        delta = Coeff_B^2 - 4 * Coeff_A * Coeff_C;

        if delta < 0
            % Should not happen if point is inside (h_curr < h_min_allowed), but handle robustly
            h_prev = h_curr; % Keep history consistent
            continue;
        end

        sqrt_delta = sqrt(delta);
        t1 = (-Coeff_B + sqrt_delta) / (2 * Coeff_A);
        t2 = (-Coeff_B - sqrt_delta) / (2 * Coeff_A);

        shift1 = t1 * n_curr;
        shift2 = t2 * n_curr;

        % --- 6. Target selection ---
        if target_dot_sign(j) > 0
            if t1 > t2, shift_vec = shift1; else, shift_vec = shift2; end
        else
            if t1 < t2, shift_vec = shift1; else, shift_vec = shift2; end
        end

        % Accumulate force
        total_shift(:, k) = total_shift(:, k) + shift_vec;
        
        % Update h_prev assuming perfect projection to the boundary
        h_prev = h_min_allowed;

    end % end k loop
end % end obs loop

% ==========================================
% 3. Apply resultant force and execute road boundary clamping
% ==========================================

% First apply collision avoidance push
z_x_temp = y_x + total_shift;

% Then execute hard clamping
if isfield(constraints, 'road_bounds') && ~isempty(constraints.road_bounds)
    y_min = constraints.road_bounds(1);
    y_max = constraints.road_bounds(2);
    z_x_temp(2, :) = max(y_min, min(y_max, z_x_temp(2, :)));
end

z_x = z_x_temp;

end