function [p, v, a] = evaluate_position(S, v0, a0, p0, t)
% EVALUATE_POSITION Evaluate position, velocity, and acceleration over a composite trajectory
% S contains: Pa (acceleration phase), Pb (deceleration phase), tpb (end of Pa), tc (cruise duration)

% 1. Acceleration phase
if t <= S.tpb
    [p, v, a] = get_state_at_t(v0, a0, p0, S.Pa, t);
    return;
end

% Compute end state of phase A for subsequent phases
% (Consider storing intermediate states in S for efficiency)
[ppb, vpb, apb] = get_state_at_t(v0, a0, p0, S.Pa, S.tpb);

% 2. Cruise phase
if t <= (S.tpb + S.tc)
    dt_cruise = t - S.tpb;
    a = 0;
    v = vpb; % constant velocity
    p = ppb + v * dt_cruise;
    return;
end

% End of cruise
p_end_cruise = ppb + vpb * S.tc;

% 3. Deceleration phase
if S.tc > 0
    a_start_b = 0; % if cruise exists, Pb starts from zero acceleration
else
    a_start_b = apb; % otherwise inherit acceleration from Pa
end

t_in_b = t - (S.tpb + S.tc);
if t_in_b > S.Pb.T3
    % Hold final state (v=0, a=0)
    [p_final, v_final, a_final] = get_state_at_t(vpb, a_start_b, p_end_cruise, S.Pb, S.Pb.T3);
    p = p_final; v = v_final; a = a_final;
else
    [p, v, a] = get_state_at_t(vpb, a_start_b, p_end_cruise, S.Pb, t_in_b);
end
end