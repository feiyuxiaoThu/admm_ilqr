"""
ADMM Optimizer - Outer ADMM Loop in Frenet Coordinates
Optimization is performed in Frenet (s, d) coordinates for curved road support
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from planning.ilqr_frenet import run_ilqr_frenet
from planning.constraints import project_constraints
from core.frenet_vehicle_model import (
    update_state_frenet,
    frenet_to_cartesian,
    cartesian_to_frenet,
    frenet_control_to_cartesian
)


def generate_initial_trajectory_frenet(x0, x_ref_traj, N, dt, L, constraints, acc_ref=None):
    """
    Generate warm-start trajectory in Frenet coordinates using simple logic

    Args:
        x0: initial state [X; Y; phi; v] (4x1 numpy array) in Cartesian
        x_ref_traj: reference trajectory (4 x N+1 numpy array) in Cartesian
        N: number of time steps (int)
        dt: time interval (float)
        L: wheelbase (float)
        constraints: dict with 'u_min', 'u_max'
        acc_ref: reference acceleration (optional, N+1 numpy array)

    Returns:
        X: state trajectory (4 x N+1 numpy array) in Cartesian
        U: control trajectory (2 x N numpy array) in Cartesian
    """
    nx = 4
    nu = 2
    U = np.zeros((nu, N))
    X = np.zeros((nx, N + 1))

    X[:, 0] = x0

    # Simple P-control for velocity
    Kp_vel = 3.0

    use_acc_ref = (acc_ref is not None) and (len(acc_ref) >= N + 1)

    # Main loop: generate control for each timestep
    for k in range(N):
        # Current state
        curr_v = X[3, k]

        # 1. Longitudinal control (P Control)
        ref_idx = min(k, x_ref_traj.shape[1] - 1)
        ref_v = x_ref_traj[3, ref_idx]

        if use_acc_ref:
            des_a = acc_ref[ref_idx]
        else:
            des_a = Kp_vel * (ref_v - curr_v)
            if (abs(ref_v) < 0.1) and (curr_v < 0.1):
                des_a = -curr_v / dt  # Quick stop

        # 2. Lateral control (simple pursuit)
        # Find target point on reference trajectory
        target_idx = min(ref_idx + 10, x_ref_traj.shape[1] - 1)
        target_x = x_ref_traj[0, target_idx]
        target_y = x_ref_traj[1, target_idx]

        # Compute desired heading
        dx = target_x - X[0, k]
        dy = target_y - X[1, k]
        des_theta = np.arctan2(dy, dx)

        # Simple steering control
        theta_error = des_theta - X[2, k]
        theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))  # Normalize to [-pi, pi]
        des_delta = theta_error * 0.5  # Simple proportional control

        # Apply control limits
        des_a = np.clip(des_a, constraints['u_min'][0], constraints['u_max'][0])
        des_delta = np.clip(des_delta, constraints['u_min'][1], constraints['u_max'][1])

        U[:, k] = [des_a, des_delta]

        # Update state
        from core.vehicle_model import update_state
        X[:, k + 1] = update_state(X[:, k], U[:, k], dt, L)

    return X, U


def run_admm_ilqr_frenet(x0, candidate, constraints, dt, L, weights, options, ego_lane):
    """
    Run ADMM-iLQR optimization in Frenet coordinates

    Args:
        x0: initial state [X, Y, phi, v] (4x1 numpy array) in Cartesian
        candidate: Candidate object with reference trajectory
        constraints: dict with constraints
        dt: time step (float)
        L: wheelbase (float)
        weights: dict with cost weights
        options: dict with algorithm options
        ego_lane: Lane object for Frenet coordinate system

    Returns:
        X_opt: optimal state trajectory in Cartesian (4 x N+1 numpy array)
        U_opt: optimal control trajectory in Cartesian (2 x N numpy array)
        debug_info: dict with debug information
    """
    N = candidate.x_ref.shape[1] - 1
    nx = 4
    nu = 2

    # Convert initial state to Frenet
    x0_frenet = cartesian_to_frenet(x0, ego_lane.reference_line)

    # Convert reference trajectory to Frenet
    x_ref_frenet = np.zeros_like(candidate.x_ref)
    for k in range(N + 1):
        x_ref_frenet[:, k] = cartesian_to_frenet(candidate.x_ref[:, k], ego_lane.reference_line)

    # Initialize trajectory
    X_init, U_init = generate_initial_trajectory_frenet(x0, candidate.x_ref, N, dt, L, constraints, candidate.acc)

    # Convert initial trajectory to Frenet
    X_frenet = np.zeros_like(X_init)
    U_frenet = np.zeros_like(U_init)
    for k in range(N + 1):
        X_frenet[:, k] = cartesian_to_frenet(X_init[:, k], ego_lane.reference_line)
    for k in range(N):
        # For control, we keep it as is (will be converted after optimization)
        U_frenet[:, k] = U_init[:, k]

    # Get curvature array
    kappa_array = np.zeros(N + 1)
    dkappa_ds_array = np.zeros(N + 1)
    for k in range(N + 1):
        s = X_frenet[0, k]
        x_ref, y_ref, theta_ref, kappa_ref = ego_lane.reference_line.get_state_at_s(s)
        kappa_array[k] = kappa_ref

        # Approximate dkappa/ds using finite difference
        if k < N:
            s_next = X_frenet[0, k + 1]
            _, _, _, kappa_next = ego_lane.reference_line.get_state_at_s(s_next)
            dkappa_ds_array[k] = (kappa_next - kappa_ref) / (s_next - s + 1e-10)
        else:
            dkappa_ds_array[k] = dkappa_ds_array[k - 1]

    # Initialize ADMM variables
    admm_data = {
        'sigma': options['sigma'],
        'z_u': np.zeros((nu, N)),
        'lambda_u': np.zeros((nu, N)),
        'z_x': np.zeros((2, N + 1)),
        'lambda_x': np.zeros((2, N + 1)),
        'z_v': np.zeros(N + 1),
        'lambda_v': np.zeros(N + 1)
    }

    cost_history = []

    # ADMM outer loop
    for admm_iter in range(options['max_admm_iter']):
        # iLQR inner loop
        X_frenet_opt, U_frenet_opt, debug_ilqr = run_ilqr_frenet(
            x0_frenet, x_ref_frenet, weights, dt, kappa_array, dkappa_ds_array,
            admm_data, options['max_ilqr_iter'], options['ilqr_tol']
        )

        if not debug_ilqr['converged']:
            debug_info = {
                'converged': False,
                'admm_iterations': admm_iter,
                'ilqr_iterations': debug_ilqr['iterations'],
                'cost_history': cost_history,
                'failed': True,
                'reason': 'iLQR did not converge'
            }
            # Convert back to Cartesian before returning
            X_opt = np.zeros_like(X_frenet_opt)
            U_opt = np.zeros_like(U_frenet_opt)
            for k in range(N + 1):
                X_opt[:, k] = frenet_to_cartesian(X_frenet_opt[:, k], ego_lane.reference_line)
            for k in range(N):
                U_opt[:, k] = frenet_control_to_cartesian(U_frenet_opt[:, k], X_frenet_opt[:, k], ego_lane.reference_line)
            return X_opt, U_opt, debug_info

        # Convert Frenet trajectory to Cartesian for constraint projection
        X_cart = np.zeros_like(X_frenet_opt)
        U_cart = np.zeros_like(U_frenet_opt)
        for k in range(N + 1):
            X_cart[:, k] = frenet_to_cartesian(X_frenet_opt[:, k], ego_lane.reference_line)
        for k in range(N):
            U_cart[:, k] = frenet_control_to_cartesian(U_frenet_opt[:, k], X_frenet_opt[:, k], ego_lane.reference_line)

        # Project constraints
        proj_result = project_constraints(U_cart, X_cart, admm_data['lambda_u'], admm_data['lambda_x'], admm_data['sigma'], constraints)
        if len(proj_result) == 3:
            z_u_new, z_x_new, z_v_new = proj_result
        else:
            z_u_new, z_x_new = proj_result
            z_v_new = X_cart[3, :].copy()  # No velocity constraint

        # Update dual variables
        lambda_u_new = admm_data['lambda_u'] + admm_data['sigma'] * (U_cart - z_u_new)
        lambda_x_new = admm_data['lambda_x'] + admm_data['sigma'] * (X_cart[:2, :] - z_x_new)
        lambda_v_new = admm_data['lambda_v'] + admm_data['sigma'] * (X_cart[3, :] - z_v_new)

        # Update ADMM variables
        admm_data['z_u'] = z_u_new
        admm_data['lambda_u'] = lambda_u_new
        admm_data['z_x'] = z_x_new
        admm_data['lambda_x'] = lambda_x_new
        admm_data['z_v'] = z_v_new
        admm_data['lambda_v'] = lambda_v_new

        # Compute cost
        from planning.ilqr_frenet import calculate_total_cost_admm_frenet
        cost = calculate_total_cost_admm_frenet(X_frenet_opt, U_frenet_opt, x_ref_frenet, weights, admm_data)
        cost_history.append(cost)

        # Check ADMM convergence
        if admm_iter > 0:
            if abs(cost_history[-1] - cost_history[-2]) < options['tol_admm']:
                break

    # Convert optimal Frenet trajectory back to Cartesian
    X_opt = np.zeros_like(X_frenet_opt)
    U_opt = np.zeros_like(U_frenet_opt)
    for k in range(N + 1):
        X_opt[:, k] = frenet_to_cartesian(X_frenet_opt[:, k], ego_lane.reference_line)
    for k in range(N):
        U_opt[:, k] = frenet_control_to_cartesian(U_frenet_opt[:, k], X_frenet_opt[:, k], ego_lane.reference_line)

    # Debug: print trajectory info
    print(f"  Frenet trajectory: s range [{X_frenet_opt[0, :].min():.2f}, {X_frenet_opt[0, :].max():.2f}], d range [{X_frenet_opt[1, :].min():.2f}, {X_frenet_opt[1, :].max():.2f}]")
    print(f"  Optimized trajectory: X range [{X_opt[0, :].min():.2f}, {X_opt[0, :].max():.2f}], Y range [{X_opt[1, :].min():.2f}, {X_opt[1, :].max():.2f}]")
    print(f"  Velocity range: [{X_opt[3, :].min():.2f}, {X_opt[3, :].max():.2f}] m/s")
    print(f"  First few points: {X_opt[:, :5]}")

    debug_info = {
        'converged': True,
        'admm_iterations': admm_iter + 1,
        'ilqr_iterations': debug_ilqr['iterations'],
        'cost_history': cost_history
    }

    return X_opt, U_opt, debug_info