function [z_u, z_x] = project_constraints_anti_distortion(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints)
% PROJECT_CONSTRAINTS_ANTI_DISTORTION ADMM step 2: Project variables onto feasible domain
%
% Inputs:
%   u_traj: current control trajectory (2 x N)
%   x_traj: current state trajectory (4 x N+1)
%   lambda_u: control dual variables (2 x N)
%   lambda_x: state dual variables (2 x N+1) -- only first two dimensions (X,Y)
%   sigma: penalty parameter
%   constraints: constraint struct
%       .u_min, .u_max: control bounds
%       .obstacles: obstacle list struct array (.x, .y, .a, .b, .theta)
%
% Outputs:
%   z_u: projected control variables (2 x N)
%   z_x: projected position variables (2 x N+1)

% ==========================================
% 1. Control constraint projection (Box Constraint)
% ==========================================
% Compute projection point y_u = u + lambda_u / sigma
y_u = u_traj + lambda_u / sigma;

% Apply clamping
z_u = max(constraints.u_min, min(constraints.u_max, y_u));

% ==========================================
% 2. Obstacle constraint projection (Obstacle Avoidance)
% ==========================================
% Extract position part (X, Y)
pos_traj = x_traj(1:2, :);

% Compute projection point y_x = x + lambda_x / sigma
y_x = pos_traj + lambda_x / sigma;

z_x = y_x; % Default: no collision

if isfield(constraints, 'obstacles') && ~isempty(constraints.obstacles)
    num_obs = length(constraints.obstacles);
    num_steps = size(y_x, 2);

    for j = 1:num_obs
        obs = constraints.obstacles(j);
        % ... (A_mat, R, a, b, center pre-computed parameters remain unchanged)
        cos_t = cos(obs.theta);
        sin_t = sin(obs.theta);
        R = [cos_t, -sin_t; sin_t, cos_t];
        a = obs.a; b = obs.b; center = [obs.x; obs.y];
        Sigma = diag([1/a^2, 1/b^2]);
        A_mat = R * Sigma * R';
        %%% --- START: Consistent Side Projection Logic --- %%%

        % 1. Identify all points colliding with this obstacle
        colliding_indices = zeros(1, num_steps);
        colliding_points_local_v = zeros(1, num_steps,1); % Store local frame v-coordinate of colliding points
        colliding_index = 0;
        for k = 1:num_steps
            pt_global = y_x(:, k);
            d_xy = pt_global - center;
            metric = d_xy' * A_mat * d_xy;

            if metric < 1.0
                colliding_index = colliding_index + 1;
                colliding_indices(1, colliding_index) = k;
                pt_local = R' * d_xy;
                colliding_points_local_v(1, colliding_index) = pt_local(2);
            end
        end
        colliding_indices = colliding_indices(1, 1:colliding_index);
        colliding_points_local_v = colliding_points_local_v(1, 1:colliding_index);

        % 2. If collisions exist, perform collective decision
        if ~isempty(colliding_indices)
            % Decision: Based on mean v-coordinate of colliding points, determine projection direction
            % (top side v>0 or bottom side v<0) to prevent oscillation between both sides of obstacle
            mean_v = mean(colliding_points_local_v);

            % If mean is near zero (piercing center), default to +v direction
            if abs(mean_v) < 1e-2
                target_side_sign = 1.0;
            else
                target_side_sign = sign(mean_v);
            end
            % 3. Project all colliding points with consistent side bias
            for k = colliding_indices
                pt_global = y_x(:, k);
                d_xy = pt_global - center;

                %% Debug: plotting routine (commented out)
                % figure; hold on; axis equal;
                % plot(pt_global(1), pt_global(2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Colliding Point');
                % % Define unit circle for transformation
                % theta_circle = linspace(0, 2*pi, 100);
                % circle_x = cos(theta_circle);
                % circle_y = sin(theta_circle);
                % % --- Ellipse transformation logic ---
                % % 1. Scale (based on semi-major axis a and semi-minor axis b)
                % scale_x = obs.a * circle_x;
                % scale_y = obs.b * circle_y;
                
                % % 2. Rotate (based on heading theta)
                % R = [cos(obs.theta), -sin(obs.theta);
                %     sin(obs.theta),  cos(obs.theta)];
                % rotated_points = R * [scale_x; scale_y];
                
                % % 3. Translate (move to center x, y)
                % final_x = rotated_points(1, :) + obs.x;
                % final_y = rotated_points(2, :) + obs.y;
                
                % % 4. Plot (fill with color)
                % fill(final_x, final_y, [1, 0.6, 0.6], ...
                %     'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 1, ...
                %     'DisplayName', 'Obstacle');
                
                % % Also plot center point for reference
                % plot(obs.x, obs.y, 'r+', 'HandleVisibility', 'off');
                %% End debug plotting routine

                % Step A: Transform to local frame
                pt_local = R' * d_xy;
                u = pt_local(1);
                v = pt_local(2);

                % Force all points to project on consistent side by modifying v-input to Newton's method
                % This "tricks" projection algorithm to treat point as being on expected side
                v = target_side_sign * abs(v);
                if abs(v) < 1e-2 % If exactly on major axis, apply small perturbation
                    v = target_side_sign * 1e-2;
                end

                % Step B: Use first-quadrant symmetry (u_abs, v_abs)
                u_abs = abs(u);
                v_abs = abs(v);

                % Symmetry Breaking: prevent local optimum when trajectory pierces ellipse center
                norm_dist = sqrt((u/a)^2 + (v/b)^2);
                % If point deeply embedded in obstacle interior (e.g., core 30% region)
                if norm_dist < 0.3
                    % Near center indicates previous iteration failed to push point out.
                    % Manually specify: push toward "easy exit" direction (short axis)!
                    if a < b
                        % x-axis is shorter, push toward x-axis
                        u_abs = max(u_abs, 0.1 * a);
                        % 0.1*a provides significant non-zero initial value for Newton's method
                    else
                        % y-axis is shorter, push toward y-axis
                        v_abs = max(v_abs, 0.1 * b);
                    end
                    
                    % If zero, assign small positive value
                    if u == 0, u = 1e-5; end
                    if v == 0, v = 1e-5; end
                end

                % Step C: Newton's method to solve for t parameter
                % Robust regularized Newton iteration with protection against edge cases
                t = 0;
                iter_max = 10; tol = 1e-4;
                limit_min = -min(a^2, b^2) + 1e-4;
                limit_max = 0;
                for iter = 1:iter_max
                    a2_t = a^2 + t; 
                    b2_t = b^2 + t;
                    % Protect against zero denominators (rare edge case, e.g., point at focus)
                    if abs(a2_t) < 1e-6
                         a2_t = 1e-6; 
                    end
                    if abs(b2_t) < 1e-6
                         b2_t = 1e-6;
                    end

                    % Compute f(t) = (a*u_abs/(a^2+t))^2 + (b*v_abs/(b^2+t))^2 - 1
                    term_x = (a * u_abs) / a2_t;
                    term_y = (b * v_abs) / b2_t;
                    f_val = term_x^2 + term_y^2 - 1;

                    % Check convergence
                    if abs(f_val) < tol
                        break;
                    end

                    % Compute f'(t) = -2 * [ (a^2*u_abs^2)/(a^2+t)^3 + (b^2*v_abs^2)/(b^2+t)^3 ]
                    df_val = -2 * ( (term_x^2)/a2_t + (term_y^2)/b2_t );

                    % Protect derivative: prevent NaN from division by zero
                    if abs(df_val) < 1e-10
                         % If derivative vanishes (typically at center), use epsilon for numerical stability
                         df_val = -1e-10; 
                    end
                    % Newton step with damping
                    t_next = t - f_val / df_val;
                    if t_next <= limit_min
                        t_next = (t + limit_min) / 2;
                    elseif t_next >= limit_max
                        t_next = (t + limit_max) / 2;
                    end
                    t = t_next;
                % Step D & E: Compute projected point in local ellipse frame and transform back to global
                denom_x = a^2 + t; denom_y = b^2 + t;
                if abs(denom_x) < 1e-8, denom_x = 1e-8; end
                if abs(denom_y) < 1e-8, denom_y = 1e-8; end
                q_x = sign(u) * (a^2 * u_abs) / denom_x;
                % v sign already corrected to match target side
                q_y = v * (b^2) / denom_y;

                Q_local = [q_x; q_y];
                Q_global = R * Q_local + center;

                % Update z_x trajectory
                z_x(:, k) = Q_global;
                
                %% Debug: plot after projection (optional)
                % plot(Q_global(1), Q_global(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Projected Point');
                % close all;
                %% End debug plotting

                % Update y_x (optional: prevent reordering issues with multiple obstacles)
                y_x(:, k) = z_x(:, k);
            end
        end
        %%% --- END: Consistent Side Projection Logic --- %%%
    end
end
end
