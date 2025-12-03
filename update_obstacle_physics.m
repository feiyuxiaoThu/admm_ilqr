function updated_list = update_obstacle_physics(obs_list, dt)
% Update real obstacle state (Ground Truth Physics)
% Currently assumes simple uniform linear motion. Modify here to test complex scenarios (e.g., front vehicle sudden deceleration)

updated_list = obs_list;
for i = 1:length(obs_list)
    % Simple kinematics update
    obs = obs_list(i);

    % Position update
    obs.x = obs.x + obs.vx * dt;
    obs.y = obs.y + obs.vy * dt;

    % (Optional) Can add variable acceleration logic here to test planner robustness
    % if i == 1 && obs.x > 60, obs.vx = max(0, obs.vx - 2.0 * dt); end % Example: front vehicle deceleration

    updated_list(i) = obs;
end
end