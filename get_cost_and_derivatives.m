function [l, lx, lu, lxx, luu, lux] = get_cost_and_derivatives(x, u, u_prev, x_ref, weights, step_admm, is_terminal)
    % GET_COST_AND_DERIVATIVES  Compute cost and derivatives (ADMM supported)
    %
    % Compact implementation consistent with the reference
    % Cost: l = 0.5 * (z - z_ref)' * Q * (z - z_ref)
    %
    % Inputs:
    %   x: state vector [X; Y; phi; v] (4x1)
    %   u: control vector [a; delta] (2x1)
    %   x_ref: reference state [X_ref; Y_ref; phi_ref; v_ref] (4x1)
    %   weights: struct containing weighting terms
    %   step_admm (optional): struct with ADMM variables. If empty, computes
    %       the original cost without ADMM terms.
    %       step_admm.sigma: penalty parameter
    %       step_admm.z_u: projected control [a_proj; delta_proj]
    %       step_admm.lambda_u: dual variable for control
    %       step_admm.z_x: projected position [X_proj; Y_proj] (for position constraints)
    %       step_admm.lambda_x: dual variable for position
    %
    % Outputs:
    %   l:   scalar cost
    %   lx:  gradient w.r.t. x (4x1)
    %   lu:  gradient w.r.t. u (2x1)
    %   lxx: Hessian w.r.t. x (4x4)
    %   luu: Hessian w.r.t. u (2x2)
    %   lux: cross Hessian (2x4)

    % 1. Calculate raw iLQR cost (tracking + control effort)

    % extract weights
    q_pos_x = weights.q_pos_x;   
    q_pos_y = weights.q_pos_y;
    q_vel = weights.q_vel;   
    r_acc = weights.r_acc;  
    r_steer = weights.r_steer; 
    q_pos_x_term = weights.q_pos_x_term;
    q_pos_y_term = weights.q_pos_y_term;
    q_vel_term = weights.q_vel_term;

    % dimensions
    nx = 4;
    nu = 2;

    % full Hessian Q for [x; u]
    if (~is_terminal)
        Q = diag([q_pos_x, q_pos_y, 0, q_vel, r_acc, r_steer]);
    else
        Q = diag([q_pos_x_term, q_pos_y_term, 0, q_vel_term, r_acc, r_steer]);
    end
    % Note: weights are used directly; for the 0.5 factor in the scalar cost
    % we keep Q as-is and apply 0.5 when computing l

    % reference and current stacked vectors
    u_ref = zeros(nu, 1);
    z_ref = [x_ref; u_ref];
    z = [x; u];

    % error vector
    dz = z - z_ref;

    % 1) scalar cost
    l = 0.5 * dz' * Q * dz;

    % 2) gradients
    lz = Q * dz;
    lx = lz(1:nx);
    lu = lz(nx+1:nx+nu);

    % 3) Hessian (constant)
    lzz = Q;
    lxx = lzz(1:nx, 1:nx);
    luu = lzz(nx+1:nx+nu, nx+1:nx+nu);
    lux = lzz(nx+1:nx+nu, 1:nx);
    
    % 4) Add control rate penalty if not terminal
    if ~is_terminal
        r_d_acc = weights.r_delta_acc;
        r_d_steer = weights.r_delta_steer;
        R_delta = diag([r_d_acc, r_d_steer]);

        delta_u = u - u_prev;

        % 1. Add to cost
        l = l + 0.5 * delta_u' * R_delta * delta_u;

        % 2. Add to gradients lu
        % d/du (0.5 * (u-up)' R (u-up)) = R * (u - up)
        lu = lu + R_delta * delta_u;

        % 3. Add to Hessian luu
        % d^2/du^2 = R
        luu = luu + R_delta;
    end
    

    % 2. Add ADMM penalty terms to cost and derivatives(Augmented Lagrangian)
    if nargin > 4 && ~isempty(step_admm)
        sigma = step_admm.sigma;

        % Handle control ADMM terms (box constraints on u)
        if isfield(step_admm, 'z_u') && ~isempty(step_admm.z_u) 
            z_u = step_admm.z_u;          % projected control (2x1)
            lambda_u = step_admm.lambda_u; % dual variable (2x1)

            % target value: u_target = z_u - lambda_u / sigma
            u_target = z_u - lambda_u / sigma;

            % error
            diff_u = u - u_target;

            % update cost: l += (sigma / 2) * ||u - u_target||^2
            l = l + (sigma / 2) * (diff_u' * diff_u);

            % update gradient: lu += sigma * (u - u_target)
            lu = lu + sigma * diff_u;

            % update Hessian: luu += sigma * I
            luu = luu + sigma * eye(nu);          
        end

        % Handle position ADMM terms (constraints on X, Y only)
        % Note: we only apply constraints to the first two state dims (X, Y)
        if isfield(step_admm, 'z_x') && ~isempty(step_admm.z_x) 
            z_x = step_admm.z_x;          % projected position (2x1)
            lambda_x = step_admm.lambda_x; % dual variable (2x1)

            % current position
            pos_curr = x(1:2);

            % target value
            pos_target = z_x - lambda_x / sigma;
            
            % error
            diff_pos = pos_curr - pos_target;

            % update cost
            l = l + (sigma / 2) * (diff_pos' * diff_pos);

            % update gradient
            lx(1:2) = lx(1:2) + sigma * diff_pos;

            % update Hessian
            lxx(1:2, 1:2) = lxx(1:2, 1:2) + sigma * eye(2);
        end
    end
end
