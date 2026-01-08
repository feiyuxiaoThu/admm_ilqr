"""
Constraint Projection - ADMM Step 2
Ported from MATLAB project_constraints.m
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np


def project_constraints(u_traj, x_traj, lambda_u, lambda_x, sigma, constraints):
    """
    ADMM Step 2: Project variables onto feasible region with anti-distortion logic

    Args:
        u_traj: Current control trajectory (2 x N numpy array)
        x_traj: Current state trajectory (4 x N+1 numpy array)
        lambda_u: Control dual variable (2 x N numpy array)
        lambda_x: State dual variable (2 x N+1 numpy array)
        sigma: ADMM penalty parameter (float)
        constraints: dict with 'u_min', 'u_max', 'obstacles'

    Returns:
        z_u: Projected control variable (2 x N numpy array)
        z_x: Projected position variable (2 x N+1 numpy array)
    """
    # 1. Control Constraint Projection (Box Constraint)
    # Formula: z = clamp(u + lambda/sigma, min, max)
    y_u = u_traj + lambda_u / sigma
    # Reshape u_min and u_max to (2, 1) for broadcasting
    u_min = constraints['u_min'].reshape(-1, 1)
    u_max = constraints['u_max'].reshape(-1, 1)
    z_u = np.maximum(u_min, np.minimum(u_max, y_u))

    # 2. Obstacle Constraint Projection (Obstacle Avoidance)
    # Extract position part (X, Y)
    pos_traj = x_traj[:2, :]

    # Compute point to project (ADMM intermediate variable)
    y_x = pos_traj + lambda_x / sigma

    # Default output as identity (no collision)
    z_x = y_x.copy()

    # Return directly if no obstacles
    if 'obstacles' not in constraints or not constraints['obstacles']:
        return z_u, z_x

    num_obs = len(constraints['obstacles'])
    num_steps = y_x.shape[1]

    for j in range(num_obs):
        obs = constraints['obstacles'][j]

        # Get obstacle position at each timestep
        if obs.prediction is not None:
            # Dynamic obstacle with prediction
            obs_x = obs.prediction[0, :]
            obs_y = obs.prediction[1, :]
            obs_theta = obs.prediction[2, :]
        else:
            # Static obstacle
            obs_x = np.full(num_steps, obs.x)
            obs_y = np.full(num_steps, obs.y)
            obs_theta = np.full(num_steps, obs.theta)

        # 2.1 Precompute obstacle geometry parameters
        a = obs.a
        b = obs.b

        # Fast detection metric A = R * Sigma * R'
        Sigma = np.diag([1 / a**2, 1 / b**2])

        # 2.2 Pass 1: Collision Detection and Global Consistency Decision
        # Purpose: Scan entire trajectory to decide which side (top/bottom) of obstacle to avoid,
        # preventing trajectory distortion.

        colliding_indices = []
        colliding_points_local_v = []

        for k in range(num_steps):
            # Get obstacle position at this timestep
            # Use min(k, len(obs_x)-1) to handle the case where num_steps > len(obs_x)
            idx = min(k, len(obs_x) - 1)
            center = np.array([obs_x[idx], obs_y[idx]])
            theta = obs_theta[idx]

            # Rotation matrix
            cos_t = np.cos(theta)
            sin_t = np.sin(theta)
            R = np.array([[cos_t, -sin_t], [sin_t, cos_t]])

            # Fast detection metric A_mat
            A_mat = R @ Sigma @ R.T

            pt_global = y_x[:, k]
            d_xy = pt_global - center

            # Ellipse equation detection: d' * A * d < 1 indicates interior
            metric = d_xy.T @ A_mat @ d_xy

            if metric < 1.0:
                colliding_indices.append(k)

                # Compute v (vertical coordinate) in local frame for direction determination
                pt_local = R.T @ d_xy
                colliding_points_local_v.append(pt_local[1])

        # 2.3 Pass 2: Execute Projection
        if colliding_indices:
            # Decision: Compute average vertical deviation
            if colliding_points_local_v:
                mean_v = np.mean(colliding_points_local_v)

                if abs(mean_v) < 1e-2:
                    target_side_sign = 1.0  # Through-center case, default upward
                else:
                    target_side_sign = np.sign(mean_v)
            else:
                target_side_sign = 1.0

            # Execute robust projection on all colliding points
            for k in colliding_indices:
                # Get obstacle position at this timestep
                # Use min(k, len(obs_x)-1) to handle the case where num_steps > len(obs_x)
                idx = min(k, len(obs_x) - 1)
                center = np.array([obs_x[idx], obs_y[idx]])
                theta = obs_theta[idx]

                # Rotation matrix
                cos_t = np.cos(theta)
                sin_t = np.sin(theta)
                R = np.array([[cos_t, -sin_t], [sin_t, cos_t]])

                pt_global = y_x[:, k]
                d_xy = pt_global - center

                # Step A: Convert to local coordinate system (u, v)
                pt_local = R.T @ d_xy
                u = pt_local[0]
                v = pt_local[1]

                # Strategy 1: Force consistent side (Anti-Distortion)
                v = target_side_sign * abs(v)
                if abs(v) < 1e-2:
                    v = target_side_sign * 1e-2

                # Strategy 2: Deep Repulsion (Deep Depenetration)
                if abs(u) < 0.1 * a:
                    u = 0.1 * a * np.sign(u) if u != 0 else 0.1 * a
                if abs(v) < 0.1 * b:
                    v = 0.1 * b * np.sign(v) if v != 0 else 0.1 * b

                u_abs = abs(u)
                v_abs = abs(v)

                # Step C: Robust Newton method for Lagrange multiplier t
                # Target equation: f(t) = (a*u / (a^2+t))^2 + (b*v / (b^2+t))^2 - 1 = 0
                t = 0
                iter_max = 10
                tol = 1e-4
                limit_min = -min(a**2, b**2) + 1e-4  # Lower bound for t
                limit_max = 0  # Interior point projection requires t < 0

                for iter_ in range(iter_max):
                    a2_t = a**2 + t
                    b2_t = b**2 + t

                    # Numerical stability protection
                    if abs(a2_t) < 1e-6:
                        a2_t = 1e-6
                    if abs(b2_t) < 1e-6:
                        b2_t = 1e-6

                    term_x = (a * u_abs) / a2_t
                    term_y = (b * v_abs) / b2_t

                    f_val = term_x**2 + term_y**2 - 1

                    if abs(f_val) < tol:
                        break

                    df_val = -2 * ((term_x**2) / a2_t + (term_y**2) / b2_t)

                    # Derivative protection for small values
                    if abs(df_val) < 1e-10:
                        df_val = -1e-10

                    # Newton iteration step
                    t_next = t - f_val / df_val

                    # Out-of-bounds protection (Bisection Fallback)
                    if t_next <= limit_min:
                        t_next = (t + limit_min) / 2
                    elif t_next >= limit_max:
                        t_next = (t + limit_max) / 2
                    t = t_next

                # Step D: Compute local projection coordinates
                denom_x = a**2 + t
                denom_y = b**2 + t

                # Prevent division by zero
                if abs(denom_x) < 1e-8:
                    denom_x = 1e-8
                if abs(denom_y) < 1e-8:
                    denom_y = 1e-8

                q_x = np.sign(u) * (a**2 * u_abs) / denom_x
                # Note: q_y sign follows previously enforced v direction
                q_y = target_side_sign * (b**2 * v_abs) / denom_y

                # Step E: Convert back to global coordinates
                Q_local = np.array([q_x, q_y])
                Q_global = R @ Q_local + center

                # Update projection result
                z_x[:, k] = Q_global

                # Update processing sequence (prevent order-dependency with multiple obstacles)
                y_x[:, k] = Q_global

    # 3. Road Boundary Constraint Projection (Box Constraint on Y)
    # Clamp Y coordinate to [y_min, y_max]
    if 'road_bounds' in constraints and constraints['road_bounds']:
        y_min = constraints['road_bounds']['y_min']
        y_max = constraints['road_bounds']['y_max']
        z_x[1, :] = np.maximum(y_min, np.minimum(y_max, z_x[1, :]))

    # 4. Velocity Constraint Projection (Box Constraint)
    # Project velocity to [v_min, v_max]
    if 'v_min' in constraints and 'v_max' in constraints:
        # Velocity is at index 3 in state trajectory
        v_traj = x_traj[3, :]
        y_v = v_traj  # No dual variable for velocity currently

        v_min = constraints['v_min']
        v_max = constraints['v_max']

        # Clamp velocity
        z_v = np.maximum(v_min, np.minimum(v_max, y_v))

        # Return z_u, z_x, z_v
        return z_u, z_x, z_v
    else:
        return z_u, z_x