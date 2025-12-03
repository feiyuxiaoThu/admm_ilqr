function [X, U] = generate_initial_trajectory(x0, x_ref_traj, N, dt, L, constraints, acc_ref)
% GENERATE_INITIAL_TRAJECTORY Generate warm-start trajectory using Pure Pursuit (lateral) and P-Control (longitudinal)
%
% Inputs:
%   x0: initial state [X; Y; phi; v]
%   x_ref_traj: reference trajectory (4 x N+1)
%   N: number of time steps
%   dt: time interval
%   L: wheelbase
%   constraints: constraint struct (.u_min, .u_max)
% Outputs:
%   X: state trajectory (4 x N+1)
%   U: control trajectory (2 x N)

nx = 4; nu = 2;
U = zeros(nu, N);
X = zeros(nx, N+1);

X(:, 1) = x0;

% --- Controller parameters ---
Kp_vel = 3.0;       % Velocity P gain
Ld_min = 5.0;       % Minimum lookahead distance (m)
kv = 1.0;           % Lookahead distance gain w.r.t. velocity (Ld = Ld_min + kv * v)

if nargin > 6 && ~isempty(acc_ref) && length(acc_ref) >= N+1
    % Use reference acceleration if provided
    use_acc_ref = true;
else
    use_acc_ref = false;
end

% Main loop: generate control for each timestep
for k = 1:N
    % Current state
    curr_x = X(1, k);
    curr_y = X(2, k);
    curr_phi = X(3, k);
    curr_v = X(4, k);

    % --- 1. Longitudinal control (P Control) ---
    % Get reference velocity at current timestep
    ref_idx = min(k, size(x_ref_traj, 2));
    ref_v = x_ref_traj(4, ref_idx);

    % Compute desired acceleration (P control)
    if use_acc_ref
        des_a = acc_ref(ref_idx);
    else
        des_a = Kp_vel * (ref_v - curr_v);
        if (abs(ref_v) < 0.1) && (curr_v < 0.1)
            des_a = -curr_v / dt; % Quick stop
        end
    end


    % --- 2. Lateral control (Pure Pursuit) ---
    % Compute dynamic lookahead distance
    Ld = Ld_min + kv * curr_v;

    % Find target point on reference trajectory
    % Strategy: search forward from ref_idx for first point at distance >= Ld
    target_idx = ref_idx;
    found_target = false;

    for j = ref_idx : size(x_ref_traj, 2)
        dx = x_ref_traj(1, j) - curr_x;
        dy = x_ref_traj(2, j) - curr_y;
        dist = sqrt(dx^2 + dy^2);

        if dist >= Ld
            target_idx = j;
            found_target = true;
            break;
        end
    end

    % If no far enough point found, use the last point
    if ~found_target
        target_idx = size(x_ref_traj, 2);
    end

    target_x = x_ref_traj(1, target_idx);
    target_y = x_ref_traj(2, target_idx);

    % Transform to vehicle local frame
    dx = target_x - curr_x;
    dy = target_y - curr_y;

    local_y = -sin(curr_phi) * dx + cos(curr_phi) * dy;

    % Actual distance (recomputed as target may have changed)
    Ld_actual = sqrt(dx^2 + dy^2);

    % Pure Pursuit: delta = atan(2 * L * sin(alpha) / Ld)
    % sin(alpha) = local_y / Ld_actual  =>  delta = atan(2 * L * local_y / Ld_actual^2)

    if Ld_actual < 1e-3 % Avoid division by zero
        des_delta = 0;
    else
        des_delta = atan((2 * L * local_y) / (Ld_actual^2));
    end

    % --- 3. Constraint saturation ---
    % Ensure initial trajectory satisfies constraints
    a_clamped = max(constraints.u_min(1), min(constraints.u_max(1), des_a));
    delta_clamped = max(constraints.u_min(2), min(constraints.u_max(2), des_delta));

    U(:, k) = [a_clamped; delta_clamped];

    % --- 4. State update ---
    X(:, k+1) = update_state(X(:, k), U(:, k), dt, L);
end
end
