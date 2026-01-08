"""
Trajectory Evaluator - Multi-criteria evaluation
Ported from MATLAB evaluate_trajectories.m
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np


def point_to_ellipse_dist(pt, center, theta, a, b):
    """
    Approximate signed distance from a point to an ellipse

    Args:
        pt: point coordinates [x, y] (2-element array)
        center: ellipse center [x, y] (2-element array)
        theta: ellipse orientation angle (float, radians)
        a: semi-major axis (float)
        b: semi-minor axis (float)

    Returns:
        dist: approximate signed distance (positive = outside, negative = inside)
    """
    # Transform to local ellipse frame
    dx = pt[0] - center[0]
    dy = pt[1] - center[1]

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    local_x = cos_t * dx + sin_t * dy
    local_y = -sin_t * dx + cos_t * dy

    # Algebraic metric = (x/a)^2 + (y/b)^2
    metric = (local_x / a)**2 + (local_y / b)**2

    # Convert to approximate Euclidean distance
    # Use conservative approximation: (sqrt(metric) - 1) * min(a,b)
    dist_approx = (np.sqrt(metric) - 1.0) * min(a, b)

    return dist_approx


def calc_min_distance_to_obstacles(traj, obstacles):
    """
    Find minimum distance to any obstacle over the trajectory

    Args:
        traj: state trajectory (4 x N+1 numpy array)
        obstacles: list of Obstacle objects

    Returns:
        min_dist: minimum distance (float)
        obs_idx: index of closest obstacle (int)
    """
    min_dist = float('inf')
    obs_idx = -1

    N = traj.shape[1]
    for k in range(N):
        ego_pos = traj[:2, k]
        for j, obs in enumerate(obstacles):
            # Get obstacle position at this timestep
            obs_pos = np.array([obs.x, obs.y])
            obs_theta = obs.theta

            d = point_to_ellipse_dist(ego_pos, obs_pos, obs_theta, obs.a, obs.b)

            if d < min_dist:
                min_dist = d
                obs_idx = j

    return min_dist, obs_idx


def calc_safety_cost(traj, obstacles, d_alert):
    """
    Compute quadratic hinge loss for safety margin

    Args:
        traj: state trajectory (4 x N+1 numpy array)
        obstacles: list of Obstacle objects
        d_alert: alert distance threshold (float)

    Returns:
        J: total safety cost (float)
    """
    J = 0
    N = traj.shape[1]

    for k in range(N):
        ego_pos = traj[:2, k]

        for obs in obstacles:
            obs_pos = np.array([obs.x, obs.y])
            obs_theta = obs.theta

            # Approximate surface distance
            dist = point_to_ellipse_dist(ego_pos, obs_pos, obs_theta, obs.a, obs.b)

            # Hinge loss: penalize only when inside the alert distance
            if dist < d_alert:
                J = J + (d_alert - dist)**2

    return J


def evaluate_trajectories(results, obstacles, last_best_id, last_best_cand_name, scenario_params, eval_weights):
    """
    Score candidate trajectories and pick the best

    Args:
        results: list of dict with 'X', 'U', 'cost', 'cand', 'debug_info'
        obstacles: list of Obstacle objects
        last_best_id: ID of previously chosen candidate (int)
        last_best_cand_name: name of previously chosen candidate (str)
        scenario_params: dict with 'v_desired' (desired speed)
        eval_weights: dict with evaluation weights

    Returns:
        best_idx: index of best trajectory in results (int)
        best_score: minimum total score (float)
        all_scores: list of dict with per-trajectory scores
    """
    best_score = float('inf')
    best_idx = -1
    num_cands = len(results)

    # Initialize debug records
    all_scores = []

    # Alert distance: inside this range safety cost may apply
    D_ALERT = 4.0  # meters

    # Collision tolerance: small negative tolerance to avoid false positives
    D_COLLISION = -0.05

    for i in range(num_cands):
        traj_X = results[i]['X']
        traj_U = results[i]['U']
        cand = results[i]['cand']

        # --- 1. Hard constraints checks ---

        # A. Collision check (pointwise)
        min_dist, _ = calc_min_distance_to_obstacles(traj_X, obstacles)

        if min_dist < D_COLLISION:
            # Collision detected -> discard
            scores = {
                'id': cand.id,
                'valid': False,
                'J_safe': float('inf'),
                'J_prog': 0,
                'J_comf': 0,
                'J_cons': 0,
                'total': float('inf')
            }
            all_scores.append(scores)
            continue

        # --- 2. Soft scoring ---

        # A. Safety margin cost (quadratic hinge loss)
        J_safety = calc_safety_cost(traj_X, obstacles, D_ALERT)

        # B. Progress & efficiency
        # Reward: distance traveled (more is better)
        dist_traveled = traj_X[0, -1] - traj_X[0, 0]
        # Penalty: velocity tracking error vs desired speed
        v_error_sq = np.sum((traj_X[3, :] - scenario_params['v_desired'])**2)

        J_progress = -eval_weights['w_progress'] * dist_traveled + eval_weights['w_ref_vel'] * v_error_sq

        # C. Comfort cost
        # J approx sum of accelerations^2 and jerk^2
        acc_sq = np.sum(traj_U[0, :]**2)
        steer_sq = np.sum(traj_U[1, :]**2)

        # Approximate jerk via finite differences
        dt = 0.1  # assumed dt
        jerk_lon = np.sum(np.diff(traj_U[0, :])**2) / dt**2
        jerk_lat = np.sum(np.diff(traj_U[1, :])**2) / dt**2

        J_comfort = (eval_weights['w_acc'] * acc_sq +
                    eval_weights['w_steer'] * steer_sq +
                    eval_weights['w_jerk_lon'] * jerk_lon +
                    eval_weights['w_jerk_lat'] * jerk_lat)

        # D. Consistency cost (penalize lane switches w.r.t. last selection)
        J_consistency = 0
        if last_best_id != -1:
            if cand.id != last_best_id:
                J_consistency = eval_weights['w_consistency']
        if last_best_cand_name:
            if cand.name != last_best_cand_name:
                J_consistency = J_consistency + eval_weights['w_consistency']

        # --- 3. Total score aggregation ---
        total_score = (eval_weights['w_safety'] * J_safety +
                      J_progress +
                      J_comfort +
                      J_consistency)

        scores = {
            'id': cand.id,
            'valid': True,
            'J_safe': J_safety,
            'J_prog': J_progress,
            'J_comf': J_comfort,
            'J_cons': J_consistency,
            'total': total_score
        }
        all_scores.append(scores)

        # Update best candidate
        if total_score < best_score:
            best_score = total_score
            best_idx = i

    return best_idx, best_score, all_scores