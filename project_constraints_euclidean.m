function [z_u, z_x] = project_constraints_euclidean(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints)
% PROJECT_CONSTRAINTS ADMM step 2: Project variables onto feasible domain
%
% Inputs:
%   u_traj: current control trajectory (2 x N)
%   x_traj: current state trajectory (4 x N+1)
%   lambda_u: control dual variables (2 x N)
%   lambda_x: state dual variables (2 x N+1) -- note: only first two dimensions (X,Y) used
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
% Note: lambda_x is also 2x(N+1) dimensional, corresponding to position only
y_x = pos_traj + lambda_x / sigma;

z_x = y_x; % Default: no change (if no collision exists)

if isfield(constraints, 'obstacles') && ~isempty(constraints.obstacles)
    num_obs = length(constraints.obstacles);
    num_steps = size(y_x, 2);
    
    for j = 1:num_obs
        obs = constraints.obstacles(j);
        
        % === 1. Build A matrix from equation (9) in paper ===
        % Note: paper's e_a, e_b correspond to semi-axis lengths
        cos_t = cos(obs.theta);
        sin_t = sin(obs.theta);
        R = [cos_t, -sin_t; sin_t, cos_t]; % Equation (7)
        
        % Obstacle parameters
        a = obs.a;
        b = obs.b;
        center = [obs.x; obs.y];
        
        % Pre-compute matrix for fast collision detection (A_mat)
        Sigma = diag([1/a^2, 1/b^2]);
        A_mat = R * Sigma * R';
        
        for k = 1:num_steps
            pt_global  = y_x(:, k);
            d_xy = pt_global  - center; % Vector difference
            
            % 1. Fast collision detection (using ellipse equation)
            metric = d_xy' * A_mat * d_xy;
            
            if metric < 1.0
                % === Collision detected: compute strict Euclidean projection ===
                
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
                
                % Step A: Transform to local coordinate frame (translation + rotation)
                pt_local = R' * d_xy;
                u = pt_local(1);
                v = pt_local(2);
                
                % Step B: Use first-quadrant symmetry to simplify solving
                u_abs = abs(u);
                v_abs = abs(v);
                
                % Symmetry Breaking: prevent local optimum when trajectory pierces ellipse center
                
                % Compute normalized radius estimate (< 1 means interior)
                norm_dist = sqrt((u/a)^2 + (v/b)^2);
                % [New strategy]: Deep repulsion
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
                    
                    % Re-assign signs (if zero, assign small positive value)
                    if u == 0, u = 1e-5; end
                    if v == 0, v = 1e-5; end
                end
                
                % [FIX 1] Singular point protection: if point very close to center, derivative = 0
                % Force small offset to allow Newton's method to compute gradient direction
                if u_abs < 1e-6 && v_abs < 1e-6
                    u_abs = 1e-5;
                    v_abs = 1e-5;
                    % Maintain original sign (if zero, default to positive)
                    if u == 0, u = 1e-5; end
                    if v == 0, v = 1e-5; end
                end
                
                % Step C: Newton's method to solve for parameter t
                % Equation: (a*u / (a^2 + t))^2 + (b*v / (b^2 + t))^2 - 1 = 0
                % Find t such that point projects to boundary.
                
                % Initial guess:
                % For interior points, root t lies in (-min(a^2, b^2), 0).
                % Simple heuristic: t = 0 (starting position, though f(0)<0, Newton usually recovers)
                % More robust initial value would use -min(a,b)*distance, but t=0 works well.
                t = 0;
                
                % Newton iteration parameters
                iter_max = 10;
                tol = 1e-4;
                
                % Hard bounds for t (for interior points)
                % t cannot be less than -min(a^2, b^2), otherwise denominator = 0 or negative
                limit_min = -min(a^2, b^2) + 1e-4; % Leave margin to prevent division by zero
                limit_max = 0; % Interior point projection: t must be negative
                
                for iter = 1:iter_max
                    a2_t = a^2 + t;
                    b2_t = b^2 + t;
                    
                    % Protect against zero denominators (rare edge case, e.g., point at focus)
                    if abs(a2_t) < 1e-6, a2_t = 1e-6; end
                    if abs(b2_t) < 1e-6, b2_t = 1e-6; end
                    
                    % Compute f(t)
                    term_x = (a * u_abs) / a2_t;
                    term_y = (b * v_abs) / b2_t;
                    f_val = term_x^2 + term_y^2 - 1;
                    
                    % Check convergence
                    if abs(f_val) < tol
                        break;
                    end
                    
                    % Compute f'(t)
                    % f'(t) = -2 * [ (a^2*u^2)/(a^2+t)^3 + (b^2*v^2)/(b^2+t)^3 ]
                    df_val = -2 * ( (term_x^2)/a2_t + (term_y^2)/b2_t );
                    % Protect derivative: prevent NaN from division by zero
                    if abs(df_val) < 1e-10
                        % If derivative vanishes (typically at center), use epsilon for stability
                        df_val = -1e-10;
                    end
                    
                    % Newton step
                    t_next = t - f_val / df_val;
                    
                    % === [Key fix]: Prevent stepping outside bounds ===
                    if t_next <= limit_min || t_next >= limit_max
                        % If Newton step overshoots, use "bisection" idea for compromise
                        % Take midpoint between current value and boundary to prevent out-of-bounds
                        if t_next <= limit_min
                            t_next = (t + limit_min) / 2;
                        else
                            t_next = (t + limit_max) / 2;
                        end
                    end
                    t = t_next;
                end
                
                % Step D: Compute projected point Q_local in local frame
                denom_x = a^2 + t;
                denom_y = b^2 + t;
                if abs(denom_x) < 1e-8, denom_x = 1e-8; end
                if abs(denom_y) < 1e-8, denom_y = 1e-8; end
                q_x_abs = (a^2 * u_abs) / denom_x;
                q_y_abs = (b^2 * v_abs) / denom_y;
                
                % Restore signs
                q_x = sign(u) * q_x_abs;
                % If u=0, sign(0)=0, so x should be 0, logic is correct
                if u == 0, q_x = 0; end
                
                q_y = sign(v) * q_y_abs;
                if v == 0, q_y = 0; end
                
                Q_local = [q_x; q_y];
                
                % Step E: Transform back to global coordinate frame
                Q_global = R * Q_local + center;
                z_x(:, k) = Q_global;
                
                %% Debug: plot after projection (optional)
                % plot(Q_global(1), Q_global(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Projected Point');
                % close all;
                %% End debug plotting
                
                % Update y_x (optional, prevents reordering issues with multiple obstacles)
                y_x(:, k) = z_x(:, k);
                
            end
        end
    end
end
end
