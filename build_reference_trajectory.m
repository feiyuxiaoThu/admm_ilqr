function [x_ref_traj] = build_reference_trajectory(scenario_id, v_target, dt, N)
% BUILD_REFERENCE_TRAJECTORY  Generate reference trajectory and initial state
%   [x_ref_traj, x0] = build_reference_trajectory(scenario_id, v_target, dt, N)
%
%   scenario_id: 1=straight, 2=circle, 3=intersection (straight->turn->straight)
%   returns x_ref_traj (4 x N+1) and initial state x0 (4x1)

x_ref_traj = zeros(4, N+1); % [X; Y; phi; v]

switch scenario_id
    case 1 % straight
        for k = 1:N+1
            t = (k-1)*dt;
            x_ref_traj(1, k) = v_target * t;    % X increases linearly
            x_ref_traj(2, k) = 0;               % Y = 0
            x_ref_traj(3, k) = 0;               % phi = 0
            x_ref_traj(4, k) = v_target;        % constant speed
        end

    case 2 % circle
        R = 20.0;
        omega = v_target / R;
        for k = 1:N+1
            t = (k-1)*dt;
            theta = omega * t;
            x_ref_traj(1, k) = R * sin(theta);
            x_ref_traj(2, k) = R - R * cos(theta);
            x_ref_traj(3, k) = theta;
            x_ref_traj(4, k) = v_target;
        end

    case 3 % intersection: straight -> left turn -> straight
        dist_straight_1 = 20.0;
        R_turn = 15.0;
        turn_angle = pi/2;

        current_dist = 0;
        for k = 1:N+1
            current_dist = current_dist + v_target * dt;

            if current_dist <= dist_straight_1
                % first straight segment
                x_ref_traj(1, k) = current_dist;
                x_ref_traj(2, k) = 0;
                x_ref_traj(3, k) = 0;
            elseif current_dist <= (dist_straight_1 + R_turn*turn_angle)
                % turning arc
                dist_in_turn = current_dist - dist_straight_1;
                theta = dist_in_turn / R_turn;

                theta_global = theta;
                x_ref_traj(1, k) = dist_straight_1 + R_turn * sin(theta_global);
                x_ref_traj(2, k) = R_turn - R_turn * cos(theta_global);
                x_ref_traj(3, k) = theta_global;
            else
                % final straight upwards
                end_x = dist_straight_1 + R_turn;
                end_y = R_turn;
                dist_in_straight_2 = current_dist - (dist_straight_1 + R_turn*turn_angle);

                x_ref_traj(1, k) = end_x;
                x_ref_traj(2, k) = end_y + dist_in_straight_2;
                x_ref_traj(3, k) = pi/2;
            end
            x_ref_traj(4, k) = v_target;
        end
    otherwise
        error('Unknown scenario_id: %d', scenario_id);
end
end
