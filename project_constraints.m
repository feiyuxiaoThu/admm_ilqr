function [z_u, z_x] = project_constraints(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints)
% PROJECT_CONSTRAINTS_ANTI_DISTORTION
% ADMM Step 2: Project variables onto feasible region with anti-distortion consistent-side logic.
%
% Inputs:
%   u_traj:      Current control trajectory (2 x N)
%   x_traj:      Current state trajectory (4 x N+1)
%   lambda_u:    Control dual variable
%   lambda_x:    State dual variable
%   sigma:       ADMM penalty parameter
%   constraints: Constraint struct (.u_min, .u_max, .obstacles)
%
% Outputs:
%   z_u: Projected control variable
%   z_x: Projected position variable

% ==========================================
% 1. Control Constraint Projection (Box Constraint)
% ==========================================
% Formula: z = clamp(u + lambda/sigma, min, max)
y_u = u_traj + lambda_u / sigma;
z_u = max(constraints.u_min, min(constraints.u_max, y_u));

% ==========================================
% 2. Obstacle Constraint Projection (Obstacle Avoidance)
% ==========================================
% Extract position part (X, Y)
pos_traj = x_traj(1:2, :);

% Compute point to project (ADMM intermediate variable)
y_x = pos_traj + lambda_x / sigma;

% Default output as identity (no collision)
z_x = y_x;

% Return directly if no obstacles
if ~isfield(constraints, 'obstacles') || isempty(constraints.obstacles)
    return;
end

num_obs = length(constraints.obstacles);
num_steps = size(y_x, 2);

for j = 1:num_obs
    obs = constraints.obstacles(j);

    % --- 2.1 Precompute obstacle geometry parameters ---
    % Rotation matrix R: transpose converts Global -> Local to Local -> Global
    cos_t = cos(obs.theta);
    sin_t = sin(obs.theta);
    R = [cos_t, -sin_t; sin_t, cos_t];

    a = obs.a;
    b = obs.b;
    center = [obs.x; obs.y];

    % Fast detection metric A = R * Sigma * R'
    Sigma = diag([1/a^2, 1/b^2]);
    A_mat = R * Sigma * R';

    % --- 2.2 Pass 1: Collision Detection and Global Consistency Decision ---
    % Purpose: Scan entire trajectory to decide which side (top/bottom) of obstacle to avoid,
    % preventing trajectory distortion.

    colliding_indices = zeros(1, num_steps);
    colliding_points_local_v = zeros(1, num_steps);
    count = 0; % Counter

    for k = 1:num_steps
        pt_global = y_x(:, k);
        d_xy = pt_global - center;

        % Ellipse equation detection: d' * A * d < 1 indicates interior
        metric = d_xy' * A_mat * d_xy;

        if metric < 1.0
            count = count + 1;
            colliding_indices(count) = k;

            % Compute v (vertical coordinate) in local frame for direction determination
            pt_local = R' * d_xy;
            colliding_points_local_v(count) = pt_local(2);
        end
    end

    % Truncate arrays to keep only valid part
    colliding_indices = colliding_indices(1:count);
    colliding_points_local_v = colliding_points_local_v(1:count);

    % --- 2.3 Pass 2: Execute Projection ---
    if count > 0
        % [Decision]: Compute average vertical deviation.
        % If mean_v > 0, most points are on top, push uniformly upward; otherwise downward.
        mean_v = mean(colliding_points_local_v);

        if abs(mean_v) < 1e-2
            target_side_sign = 1.0; % Through-center case, default upward
        else
            target_side_sign = sign(mean_v);
        end

        % Execute robust projection on all colliding points
        for k = colliding_indices
            pt_global = y_x(:, k);
            d_xy = pt_global - center;

            %% Debug plotting code
            % figure; hold on; axis equal;
            % plot(pt_global(1), pt_global(2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Colliding Point');
            % % Define unit circle for transformation
            % theta_circle = linspace(0, 2*pi, 100);
            % circle_x = cos(theta_circle);
            % circle_y = sin(theta_circle);
            % % --- Ellipse transformation logic ---
            % % 1. Scale (by semi-major axis a and semi-minor axis b)
            % scale_x = obs.a * circle_x;
            % scale_y = obs.b * circle_y;

            % % 2. Rotate (by heading theta)
            % R = [cos(obs.theta), -sin(obs.theta);
            %     sin(obs.theta),  cos(obs.theta)];
            % rotated_points = R * [scale_x; scale_y];

            % % 3. Translate (move to center x, y)
            % final_x = rotated_points(1, :) + obs.x;
            % final_y = rotated_points(2, :) + obs.y;

            % % 4. Plot (using fill for color)
            % plot(final_x, final_y, 'r-', 'LineWidth', 1, 'DisplayName', 'Obstacle');

            % % Also plot center point for observation
            % plot(obs.x, obs.y, 'r+', 'HandleVisibility', 'off');
            %% End debug plotting code


            % Step A: Convert to local coordinate system (u, v)
            pt_local = R' * d_xy;
            u = pt_local(1);
            v = pt_local(2);
            % [Strategy 1]: Force consistent side (Anti-Distortion)
            % Regardless of current point's v sign, force it to target sign direction.
            % This "tricks" subsequent Newton method to search closest point in unified direction.
            v = target_side_sign * abs(v);
            if abs(v) < 1e-2
                v = target_side_sign * 1e-2;
            end

            % Step B: Leverage first-quadrant symmetry for simplified computation

            % [Strategy 2]: Deep Repulsion (Deep Depenetration)
            % If point is deeply penetrated (norm_dist < 0.3), standard projection may not converge.
            % Core idea: determine which axis point is closer to in normalized coords,
            % then push strongly in direction of farther axis.
            % norm_dist = sqrt((u/a)^2 + (v/b)^2);
            % if norm_dist < 0.3
            %     if a < b
            %         u = max(abs(u), 0.1 * a) * sign(u); % Push toward x
            %     else
            %         v = max(abs(v), 0.1 * b) * sign(v); % Push toward y
            %     end
            %     % Sign protection
            %     if u == 0, u = 1e-5; end
            %     if v == 0, v = 1e-5; end
            % end
            
            if abs(u) < 0.1 * a
                u = 0.1 * a * sign(u);
            end
            if abs(v) < 0.1 * b
                v = 0.1 * b * sign(v);
            end




            u_abs = abs(u);
            v_abs = abs(v);


            % Step C: Robust Newton method for Lagrange multiplier t
            % Target equation: f(t) = (a*u / (a^2+t))^2 + (b*v / (b^2+t))^2 - 1 = 0
            t = 0;
            iter_max = 10;
            tol = 1e-4;
            limit_min = -min(a^2, b^2) + 1e-4; % Lower bound for t (denominators cannot be zero)
            limit_max = 0;                     % Interior point projection requires t < 0

            for iter = 1:iter_max
                a2_t = a^2 + t;
                b2_t = b^2 + t;

                % Numerical stability protection
                if abs(a2_t) < 1e-6, a2_t = 1e-6; end
                if abs(b2_t) < 1e-6, b2_t = 1e-6; end

                term_x = (a * u_abs) / a2_t;
                term_y = (b * v_abs) / b2_t;

                f_val = term_x^2 + term_y^2 - 1;

                if abs(f_val) < tol
                    break;
                end

                df_val = -2 * ( (term_x^2)/a2_t + (term_y^2)/b2_t );

                % Derivative protection for small values
                if abs(df_val) < 1e-10, df_val = -1e-10; end

                % Newton iteration step
                t_next = t - f_val / df_val;

                % Out-of-bounds protection (Bisection Fallback)
                if t_next <= limit_min
                    t_next = (t + limit_min) / 2;
                elseif t_next >= limit_max
                    t_next = (t + limit_max) / 2;
                end
                t = t_next;
            end

            % Step D: Compute local projection coordinates
            denom_x = a^2 + t;
            denom_y = b^2 + t;

            % Prevent division by zero
            if abs(denom_x) < 1e-8, denom_x = 1e-8; end
            if abs(denom_y) < 1e-8, denom_y = 1e-8; end

            q_x = sign(u) * (a^2 * u_abs) / denom_x;
            % Note: q_y sign follows previously enforced v direction
            q_y = target_side_sign * (b^2 * v_abs) / denom_y;

            % Step E: Convert back to global coordinates
            Q_local = [q_x; q_y];
            Q_global = R * Q_local + center;

            % Update projection result
            z_x(:, k) = Q_global;


            %% Debug plotting after collision projection
            % plot(Q_global(1), Q_global(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Projected Point');
            % close all;
            %% End debug plotting code

            % Update processing sequence (prevent order-dependency with multiple obstacles)
            y_x(:, k) = Q_global;
        end
    end
end
end
