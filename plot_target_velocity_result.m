function plot_target_velocity_result(P, v0, a0, p0, limits, total_time, dt)
% PLOT_TARGET_VELOCITY_RESULT - Visualize velocity planner solver results
%
% Inputs:
%   P: output struct from target_velocity_solver (with fields T1, T2, T3, j1, j2, j3)
%   v0, a0, p0: initial state (velocity, acceleration, position)
%   limits: constraints struct for plot boundaries (optional, with a_max, a_min, j_max, j_min)
%   total_time: visualization time horizon (s, default: P.T3 * 1.2)
%   dt: sampling time step (s, default: 0.01)

if nargin < 4, p0 = 0; end
if nargin < 6 || isempty(total_time), total_time = P.T3 * 1.2; end
if nargin < 7, dt = 0.01; end

time_vec = 0:dt:total_time;
N = length(time_vec);

p_hist = zeros(1, N);
v_hist = zeros(1, N);
a_hist = zeros(1, N);
j_hist = zeros(1, N);

% Sample trajectory at each time step
for k = 1:N
    t = time_vec(k);
    % Get state using three-phase trajectory model
    [p, v, a] = get_state_at_t(v0, a0, p0, P, t);
    
    p_hist(k) = p;
    v_hist(k) = v;
    a_hist(k) = a;
    
    % Get jerk value based on phase
    if t <= P.T1
        j_hist(k) = P.j1;
    elseif t <= P.T2
        j_hist(k) = P.j2;  % typically zero for constant acceleration phase
    elseif t <= P.T3
        j_hist(k) = P.j3;
    else
        j_hist(k) = 0;
    end
end

% Visualization
figure('Name', 'Target Velocity Solver Result', 'Color', 'w');
sgtitle('Target Velocity Profile (Phase A -> Cruise -> Phase B)');

% Position subplot
subplot(4, 1, 1);
plot(time_vec, p_hist, 'LineWidth', 1); grid on;
ylabel('Position (m)');
xline(P.T3, 'r--', 'End time');

% Velocity subplot
subplot(4, 1, 2);
plot(time_vec, v_hist, 'LineWidth', 1); grid on;
ylabel('Velocity (m/s)');
yline(v0, 'g--', 'Initial velocity');
yline(v_hist(end), 'b--', 'Final velocity');

% Acceleration subplot
subplot(4, 1, 3);
plot(time_vec, a_hist, 'LineWidth', 1); grid on;
ylabel('Acceleration (m/s^2)');
if nargin >= 5
    yline(limits.a_max, 'k:'); yline(limits.a_min, 'k:');
end

% Jerk subplot (piecewise constant)
subplot(4, 1, 4);
stairs(time_vec, j_hist, 'LineWidth', 1); grid on;
ylabel('Jerk (m/s^3)');
xlabel('Time (s)');
if nargin >= 5
    yline(limits.j_max, 'k:'); yline(limits.j_min, 'k:');
end