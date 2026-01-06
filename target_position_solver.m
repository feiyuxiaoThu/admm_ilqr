function [Pa, Pb, tpb, tc, total_time] = target_position_solver(p0, v0, a0, pf, vf, limits, vmax, vmin)
% Inputs:  State p0, v0, a0; target pf; target velocity vf; limits; velocity bounds vmax/vmin
% Outputs: Pa (first phase), Pb (second phase), tpb (switch time), tc (cruise time)

% --- Line 3: Compute trajectory to reach vf directly (Base Trajectory) ---
% Changed: target velocity from 0 to vf
P = target_velocity_solver(v0, a0, vf, limits);

% --- Line 4: Get state at the end of this base trajectory ---
[psp, ~, ~] = get_state_at_t(v0, a0, p0, P, P.T3);

% --- Line 5: Compute position direction dp ---
dp = sign(pf - psp);
if abs(pf - psp) < 1e-9, dp = 0; end

% --- Line 6-12: Determine cruise velocity vc ---
if dp == 1
    vc = vmax;
elseif dp == -1
    vc = vmin;
else
    vc = 0;
end

% --- Line 13: Compute trajectory Pa accelerating to vc ---
Pa = target_velocity_solver(v0, a0, vc, limits);

% --- Line 14: Get end state of Pa (pfa, v, a) ---
[pfa, v_actual_cruise, ~] = get_state_at_t(v0, a0, p0, Pa, Pa.T3);

% --- Line 15: Compute trajectory Pb from vc to vf ---
% Changed: target velocity from 0 to vf
Pb = target_velocity_solver(v_actual_cruise, 0, vf, limits);

% --- Line 16: Get Pb end state pfb (starting position is pfa) ---
[pfb, ~, ~] = get_state_at_t(v_actual_cruise, 0, pfa, Pb, Pb.T3);

% --- Line 18: Check for undershoot (Need Cruise) ---
% Logic: if (final position - target) has opposite sign to dp, haven't reached yet
if sign(pfb - pf) * dp <= 0
    % --- Line 19: Compute cruise time tc ---
    if abs(v_actual_cruise) > 1e-9
        tc = (pf - pfb) / v_actual_cruise;
    else
        tc = 0;
    end
    % --- Line 20: Set switch time tpb as full length of Pa ---
    tpb = Pa.T3;

else
    % --- Line 21-24: Overshoot case, need binary search ---
    tc = 0;
    tH = Pa.T3; % Upper bound
    tL = 0;     % Lower bound

    N = 50;        % Number of iterations
    epsilon = 1e-3; % Convergence threshold

    % --- Line 25: Main loop ---
    for counter = 1:N
        % --- Line 26: Bisect time tpb ---
        tpb = (tH + tL) / 2;

        % --- Line 27: Get Pa state at time tpb (ppb, vpb, apb) ---
        [ppb, vpb, apb] = get_state_at_t(v0, a0, p0, Pa, tpb);

        % --- Line 28: Recompute trajectory Pb from cutoff point state ---
        % Changed: target velocity from 0 to vf
        Pb = target_velocity_solver(vpb, apb, vf, limits);

        % --- Line 29: Compute new final position pfb (starting position is ppb) ---
        [pfb, ~, ~] = get_state_at_t(vpb, apb, ppb, Pb, Pb.T3);

        % --- Line 30-34: Update bisection bounds ---
        if sign(pfb - pf) * dp < 0
            tL = tpb; % Undershoot, need more Pa
        else
            tH = tpb; % Overshoot, need less Pa
        end

        % --- Line 35-37: Check convergence ---
        if abs(pfb - pf) < epsilon
            break;
        end
    end
end

total_time = tpb + tc + Pb.T3;
end