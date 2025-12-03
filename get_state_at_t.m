function [p, v, a] = get_state_at_t(v0, a0, p0, P, t_query)
    % GET_STATE_AT_T - Compute state (position, velocity, acceleration) at time t
    %
    % Integrates the trajectory profile P with three jerk phases from initial state
    % (v0, a0, p0) to the query time t_query.
    %
    % Inputs:
    %   v0, a0, p0: initial velocity, acceleration, position (scalars)
    %   P: trajectory struct with fields [T1, T2, T3, j1, j2, j3] 
    %      (phase durations and jerk values)
    %   t_query: time at which to evaluate the state (scalar)
    %
    % Outputs:
    %   p: position at time t_query
    %   v: velocity at time t_query
    %   a: acceleration at time t_query
    
    % Initialize state and remaining time
    dur_1 = P.T1;
    dur_2 = P.T2 - P.T1;
    dur_3 = P.T3 - P.T2; 
    
    p = p0; v = v0; a = a0;
    t_rem = t_query;
    
    % Phase 1: Integrate with jerk j1
    if t_rem > 0
        dt = min(t_rem, dur_1);
        p = p + v*dt + 0.5*a*dt^2 + (1/6)*P.j1*dt^3;
        v = v + a*dt + 0.5*P.j1*dt^2;
        a = a + P.j1*dt;
        t_rem = t_rem - dt;
    end
    
    % Phase 2: Integrate with jerk j2 (typically zero for constant acceleration)
    if t_rem > 0
        dt = min(t_rem, dur_2);
        p = p + v*dt + 0.5*a*dt^2 + (1/6)*P.j2*dt^3;
        v = v + a*dt + 0.5*P.j2*dt^2;
        a = a + P.j2*dt;
        t_rem = t_rem - dt;
    end
    
    % Phase 3: Integrate with jerk j3
    if t_rem > 0
        dt = min(t_rem, dur_3);
        p = p + v*dt + 0.5*a*dt^2 + (1/6)*P.j3*dt^3;
        v = v + a*dt + 0.5*P.j3*dt^2;
        a = a + P.j3*dt;
        t_rem = t_rem - dt;
    end
    
    % If time exceeds trajectory duration, continue with constant acceleration
    if t_rem > 0
        p = p + v * t_rem;
    end
end