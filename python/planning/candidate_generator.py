"""
Candidate Trajectory Generator - Frenet Coordinate System
Generates candidates in Frenet coordinates (s, d) for curved road support
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry.reference_line import ReferenceLineSystem


class Candidate:
    """Represents a candidate trajectory"""

    def __init__(self, cand_id, name, x_ref, v_target, acc, target_lane_id):
        """
        Initialize candidate

        Args:
            cand_id: unique identifier (int)
            name: descriptive name (str)
            x_ref: reference trajectory (4 x N+1 numpy array)
            v_target: target velocity (float)
            acc: acceleration profile (N+1 numpy array)
            target_lane_id: target lane index (int)
        """
        self.id = cand_id
        self.name = name
        self.x_ref = x_ref
        self.v_target = v_target
        self.acc = acc
        self.target_lane_id = target_lane_id


def find_leading_obstacle(ego_s, ego_lane, obstacles, lookahead_dist=100.0):
    """
    Find nearest obstacle ahead in specified lane using Frenet coordinates

    Args:
        ego_s: ego vehicle s-coordinate (float)
        ego_lane: Lane object representing the ego lane
        obstacles: list of Obstacle objects
        lookahead_dist: maximum distance to search ahead (float)

    Returns:
        found: boolean indicating if obstacle was found
        obs: Obstacle object if found, None otherwise
        obs_s: s-coordinate of obstacle if found
    """
    min_dist = float('inf')
    found = False
    obs = None
    obs_s = None
    lane_threshold = 2.0  # Lateral distance threshold (meters)

    for o in obstacles:
        # Project obstacle to ego lane
        s_obs, d_obs = ego_lane.reference_line.project_point(o.x, o.y)

        # Check if ahead (within lookahead distance)
        if 0 < (s_obs - ego_s) < lookahead_dist:
            # Check lateral distance
            if abs(d_obs) < lane_threshold:
                dist = s_obs - ego_s
                if dist < min_dist:
                    min_dist = dist
                    obs = o
                    obs_s = s_obs
                    found = True

    return found, obs, obs_s


def get_state_at_t(v0, a0, s0, P, t):
    """
    Get state at time t for velocity-mode trajectory (in Frenet coordinates)

    Args:
        v0: initial velocity (float)
        a0: initial acceleration (float)
        s0: initial s-position (float)
        P: velocity solver parameters (dict with 'a_max', 'v_target', 't1', 't2', 't3')
        t: time (float)

    Returns:
        s_val: s-position (float)
        v_val: velocity (float)
        a_val: acceleration (float)
    """
    a_max = P['a_max']
    v_target = P['v_target']
    t1 = P['t1']
    t2 = P['t2']
    t3 = P['t3']

    if t < t1:
        # Acceleration phase
        a_val = a_max
        v_val = v0 + a_max * t
        s_val = s0 + v0 * t + 0.5 * a_max * t**2
    elif t < t2:
        # Constant velocity phase
        a_val = 0
        v1 = v0 + a_max * t1
        s1 = s0 + v0 * t1 + 0.5 * a_max * t1**2
        v_val = v1
        s_val = s1 + v1 * (t - t1)
    elif t < t3:
        # Deceleration phase
        a_val = -a_max
        v2 = v0 + a_max * t1
        s2 = s0 + v0 * t1 + 0.5 * a_max * t1**2
        v_val = v2 - a_max * (t - t2)
        s_val = s2 + v2 * (t2 - t1) - 0.5 * a_max * (t - t2)**2
    else:
        # Constant velocity at target
        a_val = 0
        v_val = v_target
        s3 = s0 + v0 * t1 + 0.5 * a_max * t1**2 + v_target * (t2 - t1) + v_target * (t3 - t2) - 0.5 * a_max * (t3 - t2)**2
        s_val = s3 + v_target * (t - t3)

    return s_val, v_val, a_val


def evaluate_position(P_struct, v0, a0, s0, t):
    """
    Get state at time t for position-mode trajectory (piecewise, in Frenet coordinates)

    Args:
        P_struct: position solver parameters (dict with 'Pa', 'Pb', 'tpb', 'tc')
        v0: initial velocity (float)
        a0: initial acceleration (float)
        s0: initial s-position (float)
        t: time (float)

    Returns:
        s_val: s-position (float)
        v_val: velocity (float)
        a_val: acceleration (float)
    """
    Pa = P_struct['Pa']
    Pb = P_struct['Pb']
    tpb = P_struct['tpb']
    tc = P_struct['tc']

    if t < tpb:
        # Phase A: acceleration to reach target position
        a_val = Pa['a']
        v_val = v0 + a_val * t
        s_val = s0 + v0 * t + 0.5 * a_val * t**2
    elif t < tc:
        # Phase B: constant velocity
        a_val = 0
        v_val = Pa['v_final']
        s_pb = s0 + v0 * tpb + 0.5 * Pa['a'] * tpb**2
        s_val = s_pb + Pa['v_final'] * (t - tpb)
    else:
        # Phase C: deceleration to stop
        a_val = Pb['a']
        v_val = Pa['v_final'] + a_val * (t - tc)
        s_pc = s0 + v0 * tpb + 0.5 * Pa['a'] * tpb**2 + Pa['v_final'] * (tc - tpb)
        s_val = s_pc + Pa['v_final'] * (t - tc) + 0.5 * a_val * (t - tc)**2

    return s_val, v_val, a_val


def discretize_trajectory_frenet(P_struct, mode, x0, ego_lane, target_lane, N, dt):
    """
    Discretize trajectory in Frenet coordinates and convert to Cartesian

    This function:
    1. Generates longitudinal motion in Frenet (s, d) coordinates
    2. Converts to Cartesian (x, y, theta) coordinates using reference line

    Args:
        P_struct: velocity or position solver parameters (dict)
        mode: 'velocity' or 'position'
        x0: initial state [X, Y, phi, v] (4x1 numpy array)
        ego_lane: Lane object representing current lane (reference for Frenet)
        target_lane: Lane object representing target lane
        N: number of time steps (int)
        dt: time step (float)

    Returns:
        traj: discretized trajectory in Cartesian coordinates (4 x N+1 numpy array)
        acc: acceleration profile (N+1 numpy array)
    """
    traj = np.zeros((4, N + 1))
    acc = np.zeros(N + 1)

    p0_x = x0[0]
    p0_y = x0[1]
    theta0 = x0[2]
    v0 = x0[3]
    a0 = 0 if len(x0) < 5 else x0[4]

    # Get initial Frenet coordinates
    s0, d0 = ego_lane.reference_line.project_point(p0_x, p0_y)

    # Lane change duration
    T_lane_change = 4.0

    # Target lateral offset (d-coordinate) for target lane
    # Project a point on target lane to ego lane to get lateral offset
    target_s, target_d = ego_lane.reference_line.project_point(
        target_lane.reference_line.waypoints[0, 0],
        target_lane.reference_line.waypoints[0, 1]
    )

    for k in range(N + 1):
        t = k * dt

        # 1. Longitudinal motion in Frenet coordinates (s, v, a)
        if mode == 'velocity':
            s_val, v_val, a_val = get_state_at_t(v0, a0, s0, P_struct, t)
        else:  # mode == 'position'
            s_val, v_val, a_val = evaluate_position(P_struct, v0, a0, s0, t)

        # Clamp s_val to valid range
        s_max = ego_lane.reference_line.get_length()
        s_val = np.clip(s_val, 0, s_max)

        # 2. Lateral motion in Frenet coordinates (d-coordinate)
        if t < T_lane_change and ego_lane != target_lane:
            # Lane change: smooth transition using quintic polynomial
            ratio = t / T_lane_change
            s_poly = 10 * ratio**3 - 15 * ratio**4 + 6 * ratio**5
            d_val = d0 + (target_d - d0) * s_poly
        else:
            # Stay on target lane
            d_val = target_d

        # 3. Convert Frenet to Cartesian coordinates
        x_val, y_val, theta_val, _ = ego_lane.reference_line.get_state_at_s(s_val)

        # Adjust for lateral offset
        # Rotate by theta_val and add lateral displacement
        x_val = x_val - d_val * np.sin(theta_val)
        y_val = y_val + d_val * np.cos(theta_val)

        traj[:, k] = [x_val, y_val, theta_val, v_val]
        acc[k] = a_val

    return traj, acc


def target_velocity_solver(v0, a0, v_target, limits):
    """
    Solve for velocity trajectory to reach target velocity

    Args:
        v0: initial velocity (float)
        a0: initial acceleration (float)
        v_target: target velocity (float)
        limits: dict with 'a_max', 'a_min', 'j_max', 'j_min'

    Returns:
        P: dict with solver parameters
    """
    a_max = limits['a_max']
    a_min = limits['a_min']

    # Simple trapezoidal velocity profile
    if v_target > v0:
        # Accelerate
        a = a_max
        t1 = (v_target - v0) / a
        t2 = t1  # No constant velocity phase for simplicity
        t3 = t2 + 1.0  # Small coast phase
    else:
        # Decelerate
        a = a_min
        t1 = (v_target - v0) / a
        t2 = t1
        t3 = t2 + 1.0

    P = {
        'a_max': a,
        'v_target': v_target,
        't1': t1,
        't2': t2,
        't3': t3
    }

    return P


def generate_candidates_structured(x0, scenario, constraints, N, dt):
    """
    Generate multimodal candidate trajectories using Frenet coordinate system

    The reference line (ego lane) is used as the Frenet coordinate system.
    Curved roads are converted to straight roads in Frenet coordinates.

    Args:
        x0: ego state [X, Y, phi, v] (4x1 numpy array)
        scenario: Scenario object with ref_line_system
        constraints: dict with constraints
        N: number of time steps (int)
        dt: time step (float)

    Returns:
        candidates: list of Candidate objects
    """
    p0_x = x0[0]
    p0_y = x0[1]
    v0 = x0[3]
    a0 = 0 if len(x0) < 5 else x0[4]

    # Physical limits
    limits = {
        'a_max': constraints['u_max'][0],
        'a_min': constraints['u_min'][0],
        'j_max': 1.0,
        'j_min': -1.0
    }

    v_max = 20.0
    v_min = 0.0
    v_desired = scenario.v_desired

    candidates = []
    cand_id = 1

    # Get reference line system from scenario
    ref_line_system = scenario.ref_line_system
    all_lanes = ref_line_system.get_all_lanes()

    # Find ego lane (reference for Frenet coordinates)
    ego_lane = ref_line_system.get_lane_by_position(p0_x, p0_y)

    if ego_lane is None:
        # Fallback: use the first lane
        ego_lane = all_lanes[0]

    # Get ego's initial Frenet coordinates
    s0, d0 = ego_lane.reference_line.project_point(p0_x, p0_y)

    # Get neighbor lanes
    left_lane = ego_lane.left_neighbor if ego_lane.left_neighbor else None
    right_lane = ego_lane.right_neighbor if ego_lane.right_neighbor else None

    # Define search directions: [current, left, right]
    search_lanes = [('Keep', ego_lane)]
    if left_lane is not None:
        search_lanes.append(('Left', left_lane))
    if right_lane is not None:
        search_lanes.append(('Right', right_lane))

    for lat_action, target_lane in search_lanes:
        # Perception: find leading obstacle in target lane
        has_lead, lead_obs, obs_s = find_leading_obstacle(s0, target_lane, constraints['obstacles'])

        # Longitudinal mode A: reach desired speed (Cruise/Overtake)
        P_cruise = target_velocity_solver(v0, a0, v_desired, limits)
        traj_cruise, acc_cruise = discretize_trajectory_frenet(P_cruise, 'velocity', x0, ego_lane, target_lane, N, dt)

        candidates.append(Candidate(cand_id, f"{lat_action}_Cruise", traj_cruise, v_desired, acc_cruise, target_lane.id))
        cand_id += 1

        # Longitudinal mode B: adapt to leading vehicle (Follow/Yield/Stop)
        if has_lead:
            v_lead = lead_obs.vx
            safe_dist = 10.0 + v_lead * 1.5  # Simple safety distance model

            if v_lead > 1.0:
                # B1: Leading vehicle moving -> match velocity
                target_v = min(v_lead, v_desired)
                P_follow = target_velocity_solver(v0, a0, target_v, limits)
                traj_follow, acc_follow = discretize_trajectory_frenet(P_follow, 'velocity', x0, ego_lane, target_lane, N, dt)

                candidates.append(Candidate(cand_id, f"{lat_action}_Follow_V", traj_follow, target_v, acc_follow, target_lane.id))
                cand_id += 1
            else:
                # B2: Leading vehicle stationary -> position-based stop
                target_s = obs_s - safe_dist

                if target_s > s0:
                    # Simplified position solver
                    Pa = {'a': limits['a_min'], 'v_final': 0}
                    tpb = (0 - v0) / limits['a_min']
                    Pb = {'a': 0}
                    tc = tpb

                    solver_out = {
                        'Pa': Pa,
                        'Pb': Pb,
                        'tpb': tpb,
                        'tc': tc
                    }

                    # Modify P_struct to use target_s
                    # For simplicity, we'll use the velocity solver to reach 0
                    # The actual stopping position will be handled by the optimizer
                    traj_stop, acc_stop = discretize_trajectory_frenet(solver_out, 'position', x0, ego_lane, target_lane, N, dt)

                    candidates.append(Candidate(cand_id, f"{lat_action}_Stop", traj_stop, 0, acc_stop, target_lane.id))
                    cand_id += 1

    return candidates