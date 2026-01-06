% --- Corrected verification and plotting script ---
close all; clear; clc;

% 1. Re-run calculation
p0 = 0; v0 = 8; a0 = 0; pf = 60;
limits.a_max = 2.0; limits.a_min = -3.0; 
limits.j_max = 1.0; limits.j_min = -1.0;
v_max = 15; v_min = 0;

[Pa, Pb, tpb, tc, T_total] = target_position_solver(p0, v0, a0, pf, limits, v_max, v_min);

fprintf('Corrected calculation: total_time=%.4fs, switch_point_tpb=%.4fs\n', T_total, tpb);

% 2. Sampling and plotting
dt = 0.01;
time_vec = 0:dt:(T_total + 0.5); % Plot a bit more to see end steady-state
s_hist = []; v_hist = []; a_hist = []; t_hist = [];

for t = time_vec
    if t <= tpb
        % === Phase 1: Pa (acceleration/adjustment) ===
        [s, v, a] = get_state_at_t(v0, a0, p0, Pa, t);
    elseif t <= tpb + tc
        % === Phase 2: Cruise ---
        % Get end state of Pa
        [s_end_a, v_end_a, a_end_a] = get_state_at_t(v0, a0, p0, Pa, tpb);
        
        t_cruise = t - tpb;
        s = s_end_a + v_end_a * t_cruise;
        v = v_end_a;
        a = 0; % Acceleration is 0 during cruise
    else
        % === Phase 3: Pb (braking/final entry) ===
        % 1. Get end state of Pa
        [s_end_a, v_end_a, a_end_a] = get_state_at_t(v0, a0, p0, Pa, tpb);
        
        % 2. Compute state after cruise (as starting point for Pb)
        s_start_b = s_end_a + v_end_a * tc;
        v_start_b = v_end_a;
        
        % Critical correction: Pb initial acceleration depends on whether cruise exists
        if tc > 0
            a_start_b = 0; % If cruise exists, start braking from 0 acceleration
        else
            a_start_b = a_end_a; % [IMPORTANT] If binary search cutoff, must inherit Pa acceleration!
        end
        
        t_in_b = t - (tpb + tc);
        
        % 3. Integrate Pb (prevent extrapolation beyond Pb planned time)
        if t_in_b > Pb.T3
             % Maintain final state (v=0, a=0)
             [s_final, v_final, a_final] = get_state_at_t(v_start_b, a_start_b, s_start_b, Pb, Pb.T3);
             s = s_final; v = 0; a = 0; 
        else
             [s, v, a] = get_state_at_t(v_start_b, a_start_b, s_start_b, Pb, t_in_b);
        end
    end
    
    s_hist(end+1) = s;
    v_hist(end+1) = v;
    a_hist(end+1) = a;
    t_hist(end+1) = t;
end

% Plotting
figure('Name', 'Corrected Trajectory');
subplot(3,1,1); plot(t_hist, s_hist, 'LineWidth', 1.5); 
hold on; yline(pf, 'r--', 'Target'); 
ylabel('Position (m)'); grid on; title(sprintf('Target: %.1fm, Final: %.2fm', pf, s_hist(end)));
xline(tpb, 'k--', 'Pa');
if tc > 1e-9
    xline(tpb+tc, 'k--', 'Cruise');
end
if Pb.T3 > 1e-9
    xline(tpb+tc+Pb.T3, 'r--', 'Pb');
end

subplot(3,1,2); plot(t_hist, v_hist, 'LineWidth', 1.5); 
ylabel('Velocity (m/s)'); grid on; 

subplot(3,1,3); plot(t_hist, a_hist, 'LineWidth', 1.5); 
ylabel('Accel (m/s^2)'); grid on; xlabel('Time (s)');