function [z_u, z_x] = project_constraints_ray_casting(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints)
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

        Sigma = diag([1/obs.a^2, 1/obs.b^2]);
        A_mat = R * Sigma * R'; % Equation (9)

        % Obstacle center
        center = [obs.x; obs.y];

        for k = 1:num_steps
            pt = y_x(:, k);
            d_xy = pt - center; % Vector difference

            % === 2. Check constraint equation (8) ===
            % h(x) = 1 - d_xy' * A * d_xy <= 0  ==>  d_xy' * A * d_xy >= 1
            metric = d_xy' * A_mat * d_xy;

            if metric < 1.0
                % === Collision (point inside ellipse) ===
                % Need to find projection point z on boundary.
                % Such that z satisfies (z-c)'*A*(z-c) = 1, and ||z - pt|| is minimized.

                % Use simplified geometric projection (still the most robust method)
                % Although collision check uses matrix A, projection uses local frame ray-casting
                % Ray-circle intersection is still the fastest numerical method for solving metric = 1.

                % Here, metric is essentially (r_local / r_boundary)^2
                % So scaling factor = 1 / sqrt(metric)

                % This simple scaling corresponds to "center ray projection"
                % z = center + d_xy / sqrt(metric)

                z_x(:, k) = center + d_xy / sqrt(metric);

                % Update y_x to handle overlapping obstacles (optional)
                y_x(:, k) = z_x(:, k);
            end
        end
    end
end
end
