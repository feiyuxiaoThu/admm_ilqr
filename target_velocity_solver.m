function P = target_velocity_solver(v0, a0, v_d, limits)
% Inputs:  v0, a0, v_d (target velocity)
%          limits contains: amax, amin, jmax, jmin
% Output:  P (contains cumulative time nodes T1, T2, T3 and jerk j1, j2, j3)

amax = limits.a_max; amin = limits.a_min;
jmax = limits.j_max; jmin = limits.j_min;

% --- Line 3-7: Compute estimated velocity ve ---
if a0 >= 0
    ve = v0 + a0 * abs(a0/jmin) / 2;
else
    ve = v0 + a0 * abs(a0/jmax) / 2;
end

% --- Line 8: Compute direction da ---
da = sign(v_d - ve);
if abs(v_d - ve) < 1e-9, da = 0; end

% --- Line 9-15: Determine cruise acceleration ac ---
if da == 1
    ac = amax;
elseif da == -1
    ac = amin;
else
    ac = 0;
end

% --- Line 16-22: Compute first phase t1, j1 ---
if (ac - a0) > 0
    t1 = (ac - a0)/jmax;
    j1 = jmax;
elseif (ac - a0) < 0
    t1 = (ac - a0)/jmin;
    j1 = jmin;
else
    t1 = 0;
    j1 = 0; % no change in acceleration needed
end

% --- Line 23: Compute velocity at t1 ---
v1 = v0 + a0*t1 + 0.5*j1*t1^2;

% --- Line 24-30: Compute third phase t3, j3 ---
if -ac > 0
    t3 = -ac/jmax;
    j3 = jmax;
elseif -ac < 0
    t3 = -ac/jmin;
    j3 = jmin;
else
    t3 = 0;
    j3 = 0; % no change in acceleration needed
end

% --- Line 31: Compute velocity change in phase 3: v3_bar ---
% Note: Original formula is v3 = ac*t3 + t3^2*j3/2
v3_bar = ac*t3 + 0.5*j3*t3^2;

% --- Line 32: Compute remaining velocity change v2_bar ---
v2_bar = v_d - v1 - v3_bar;

% --- Line 33-37: Compute constant acceleration phase time t2 ---
if da == 0
    t2 = 0;
elseif abs(ac) < 1e-9 
    % If need to accelerate (da!=0) but physical limit is 0 (ac=0), can't accelerate.
    % Check if this is due to "overshoot" (though ac=0, v2_bar may be opposite direction):
    if (v2_bar * da < 0)
        t2 = -1; % Force trigger subsequent overshoot correction (Solver logic)
    else
        t2 = 0;  % Can't accelerate, only maintain current state (Best Effort)
    end
else
    t2 = v2_bar / ac;
end

% --- Line 38: Check if t2 < 0 (overshoot handling) ---
if t2 < 0
    if da == 1
        % --- Line 40-43: Acceleration overshoot, correct an ---
        % Formula: sqrt( (2*(vd-v0) + a0^2/jmax) / (1/jmax - 1/jmin) )
        numerator = 2*(v_d - v0) + a0^2/jmax;
        denominator = 1/jmax - 1/jmin;
        an = sqrt(numerator / denominator);

        t1 = (an - a0)/jmax;
        t2 = 0;
        t3 = -an/jmin;

        % Update j1, j3 to match corrected trajectory
        j1 = jmax;
        j3 = jmin;

    elseif da == -1
        % --- Line 45-48: Deceleration overshoot, correct an ---
        % Formula: -sqrt( (2*(vd-v0) + a0^2/jmin) / (1/jmin - 1/jmax) )
        numerator = 2*(v_d - v0) + a0^2/jmin;
        denominator = 1/jmin - 1/jmax;
        an = -sqrt(numerator / denominator);

        t1 = (an - a0)/jmin;
        t2 = 0;
        t3 = -an/jmax;

        % Update j1, j3
        j1 = jmin;
        j3 = jmax;
    end
end

% --- Line 51-56: Build output P (cumulative time) ---
P.T1 = t1;
P.T2 = t1 + t2;
P.T3 = t1 + t2 + t3; % Total time
P.j1 = j1;
P.j2 = 0;
P.j3 = j3;

% Store individual phase durations for getState convenience,
P.t1 = t1; P.t2 = t2; P.t3 = t3;
end