function plot_target_position_result(S, p0, v0, a0, limits, dt, plot_time, target_pos)
% PLOT_TARGET_POSITION_RESULT - Visualize position planner solver results
%
% Inputs:
%   S: struct with solver output fields (S.Pa, S.Pb, S.tpb, S.tc)
%   p0, v0, a0: initial state (position, velocity, acceleration)
%   limits: constraints struct (optional, with a_max, a_min, j_max, j_min)
%   dt: sampling time step (s)
%   plot_time: time horizon for visualization (s)
%   target_pos: target position for reference line

% Unpack solver output
Pa = S.Pa;
Pb = S.Pb;
tpb = S.tpb;
tc = S.tc;

time_vec = 0:dt:plot_time;
N = length(time_vec);

p_hist = zeros(1, N);
v_hist = zeros(1, N);
a_hist = zeros(1, N);
j_hist = zeros(1, N);

% Pre-compute phase boundaries for multi-segment trajectory
% Phase A endpoint (at time tpb)
[ppb, vpb, apb] = get_state_at_t(v0, a0, p0, Pa, tpb);

% Cruise phase endpoint (after constant velocity segment)
p_cruise_end = ppb + vpb * tc;

for k = 1:N
    t = time_vec(k);

    if t <= tpb
        % Phase A: Acceleration/adjustment phase
        [p, v, a] = get_state_at_t(v0, a0, p0, Pa, t);

        % Lookup jerk value based on phase within Pa
        if t <= Pa.T1, j = Pa.j1; %#ok<ALIGN>
        elseif t <= Pa.T2, j = Pa.j2;
        elseif t <= Pa.T3, j = Pa.j3;
        else, j = 0; end

    elseif t <= (tpb + tc)
        % Phase C: Constant velocity cruise phase
        dt_cruise = t - tpb;
        p = ppb + vpb * dt_cruise;
        v = vpb;
        a = 0;
        j = 0;
    else
        % Phase B: Deceleration/adjustment phase
        if tc > 0
            a_start_b = 0;  % Start from zero acceleration after cruise
        else
            a_start_b = apb;  % Inherit acceleration from phase A
        end
        t_in_b = t - (tpb + tc);

        if t_in_b > Pb.T3
            % Beyond phase B, maintain final state
            [p_final, v_final, a_final] = get_state_at_t(vpb, a_start_b, p_cruise_end, Pb, Pb.T3);
            p = p_final;
            v = v_final;
            a = a_final;
            j = 0;
        else
            [p, v, a] = get_state_at_t(vpb, a_start_b, p_cruise_end, Pb, t_in_b);
            if t_in_b <= Pb.T1, j = Pb.j1; %#ok<ALIGN>
            elseif t_in_b <= Pb.T2, j = Pb.j2;
            elseif t_in_b <= Pb.T3, j = Pb.j3;
            else, j = 0; end
        end
    end

    p_hist(k) = p;
    v_hist(k) = v;
    a_hist(k) = a;
    j_hist(k) = j;
end

% Visualization
figure('Name', 'Target Position Solver Result', 'Color', 'w');
sgtitle('Target Position Profile (Phase A -> Cruise -> Phase B)');

% Position subplot
subplot(4, 1, 1);
plot(time_vec, p_hist, 'LineWidth', 1); grid on;
ylabel('Position (m)');
if tpb < plot_time
    xline(tpb, 'k--', 'Phase A end');
end
if tc > 1e-9 && (tpb + tc) < plot_time
    xline(tpb+tc, 'k--', 'Cruise end');
end
if Pb.T3 > 1e-9 && (tpb + tc + Pb.T3) < plot_time
    xline(tpb+tc+Pb.T3, 'r--', 'Phase B end');
end
yline(target_pos, 'r--', 'Target');

% Velocity subplot
subplot(4, 1, 2);
plot(time_vec, v_hist, 'LineWidth', 1); grid on;
ylabel('Velocity (m/s)');

% Acceleration subplot
subplot(4, 1, 3);
plot(time_vec, a_hist, 'LineWidth', 1); grid on;
ylabel('Acceleration (m/s^2)');
if nargin >= 5
    yline(limits.a_max, 'k:'); yline(limits.a_min, 'k:');
end

% Jerk subplot
subplot(4, 1, 4);
stairs(time_vec, j_hist, 'LineWidth', 1); grid on;
ylabel('Jerk (m/s^3)');
xlabel('Time (s)');
if nargin >= 5
    yline(limits.j_max, 'k:'); yline(limits.j_min, 'k:');
end
end