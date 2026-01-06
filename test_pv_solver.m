% --- Modified verification script for vf != 0 ---
close all; clear; clc;

% 1. Setup Parameters
p0 = 0; v0 = 10; a0 = 0; 
pf = 50;      % Target Position
vf = 0;        % Target Velocity [NEW]

limits.a_max = 2.0; limits.a_min = -3.0; 
limits.j_max = 1.0; limits.j_min = -1.0;
v_max = 15; v_min = 0;

% 2. Run Solver
% Note: Added vf to the input arguments
[Pa, Pb, tpb, tc, T_total] = target_position_solver(p0, v0, a0, pf, vf, limits, v_max, v_min);

fprintf('Result: Total Time=%.4fs, Switch Time tpb=%.4fs, Cruise Time tc=%.4fs\n', T_total, tpb, tc);

% 3. Sampling and plotting
dt = 0.01;
time_vec = 0:dt:(T_total + 1.0); 
s_hist = []; v_hist = []; a_hist = []; t_hist = [];

for t = time_vec
    if t <= tpb
        % === Phase 1: Pa ===
        [s, v, a] = get_state_at_t(v0, a0, p0, Pa, t);
    elseif t <= tpb + tc
        % === Phase 2: Cruise ===
        [s_end_a, v_end_a, ~] = get_state_at_t(v0, a0, p0, Pa, tpb);
        t_cruise = t - tpb;
        s = s_end_a + v_end_a * t_cruise;
        v = v_end_a;
        a = 0;
    else
        % === Phase 3: Pb ===
        [s_end_a, v_end_a, a_end_a] = get_state_at_t(v0, a0, p0, Pa, tpb);
        s_start_b = s_end_a + v_end_a * tc;
        v_start_b = v_end_a;
        
        if tc > 0
            a_start_b = 0;
        else
            a_start_b = a_end_a; 
        end
        
        t_in_b = t - (tpb + tc);
        if t_in_b > Pb.T3
             % Maintain final state (constant velocity vf)
             [s_final, ~, ~] = get_state_at_t(v_start_b, a_start_b, s_start_b, Pb, Pb.T3);
             % After T3, velocity is vf, accel is 0
             s = s_final + vf * (t_in_b - Pb.T3); 
             v = vf; 
             a = 0; 
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
figure('Name', 'Trajectory with Final Velocity');
subplot(3,1,1); plot(t_hist, s_hist, 'LineWidth', 1.5); 
hold on; yline(pf, 'r--', 'Target Pos'); 
ylabel('Position (m)'); grid on; title(sprintf('Target: %.1fm (vf=%.1f), Final: %.2fm', pf, vf, s_hist(end)));

subplot(3,1,2); plot(t_hist, v_hist, 'LineWidth', 1.5); 
hold on; yline(vf, 'g--', 'Target Vel'); yline(v_max, 'k:', 'V Max');
ylabel('Velocity (m/s)'); grid on; 

subplot(3,1,3); plot(t_hist, a_hist, 'LineWidth', 1.5); 
ylabel('Accel (m/s^2)'); grid on; xlabel('Time (s)');