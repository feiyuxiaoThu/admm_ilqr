"""
ADMM Optimizer - Outer ADMM Loop
Ported from MATLAB run_admm_ilqr.m
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from planning.ilqr import forward_pass_admm, backward_pass_admm
from planning.constraints import project_constraints
from core.vehicle_model import update_state


def generate_initial_trajectory(x0, x_ref_traj, N, dt, L, constraints, acc_ref=None):
    """
    Generate warm-start trajectory using Pure Pursuit (lateral) and P-Control (longitudinal)
    Ported from MATLAB generate_initial_trajectory.m

    Args:
        x0: initial state [X; Y; phi; v] (4x1 numpy array)
        x_ref_traj: reference trajectory (4 x N+1 numpy array)
        N: number of time steps (int)
        dt: time interval (float)
        L: wheelbase (float)
        constraints: dict with 'u_min', 'u_max'
        acc_ref: reference acceleration (optional, N+1 numpy array)

    Returns:
        X: state trajectory (4 x N+1 numpy array)
        U: control trajectory (2 x N numpy array)
    """
    nx = 4
    nu = 2
    U = np.zeros((nu, N))
    X = np.zeros((nx, N + 1))

    X[:, 0] = x0

    # Controller parameters
    Kp_vel = 3.0       # Velocity P gain
    Ld_min = 5.0       # Minimum lookahead distance (m)
    kv = 1.0           # Lookahead distance gain w.r.t. velocity

    use_acc_ref = (acc_ref is not None) and (len(acc_ref) >= N + 1)

    # Main loop: generate control for each timestep
    for k in range(N):
        # Current state
        curr_x = X[0, k]
        curr_y = X[1, k]
        curr_phi = X[2, k]
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

        # 2. Lateral control (Pure Pursuit)
        Ld = Ld_min + kv * curr_v

        # Find target point on reference trajectory
        target_idx = ref_idx
        found_target = False

        for j in range(ref_idx, x_ref_traj.shape[1]):
            dx = x_ref_traj[0, j] - curr_x
            dy = x_ref_traj[1, j] - curr_y
            dist = np.sqrt(dx**2 + dy**2)

            if dist >= Ld:
                target_idx = j
                found_target = True
                break

        if not found_target:
            target_idx = x_ref_traj.shape[1] - 1

        target_x = x_ref_traj[0, target_idx]
        target_y = x_ref_traj[1, target_idx]

        # Transform to vehicle local frame
        dx = target_x - curr_x
        dy = target_y - curr_y

        local_y = -np.sin(curr_phi) * dx + np.cos(curr_phi) * dy

        Ld_actual = np.sqrt(dx**2 + dy**2)

        # Pure Pursuit: delta = atan(2 * L * sin(alpha) / Ld)
        if Ld_actual < 1e-3:
            des_delta = 0
        else:
            des_delta = np.arctan((2 * L * local_y) / (Ld_actual**2))

        # 3. Constraint saturation
        a_clamped = np.maximum(constraints['u_min'][0], np.minimum(constraints['u_max'][0], des_a))
        delta_clamped = np.maximum(constraints['u_min'][1], np.minimum(constraints['u_max'][1], des_delta))

        U[:, k] = [a_clamped, delta_clamped]

        # 4. State update
        X[:, k + 1] = update_state(X[:, k], U[:, k], dt, L)

    return X, U


def run_admm_ilqr(x0, candidate, constraints, dt, L, weights, options):
    """
    Run ADMM-iLQR optimization for a single candidate trajectory

    Args:
        x0: initial state [X; Y; phi; v] (4x1 numpy array)
        candidate: Candidate object with x_ref trajectory
        constraints: dict with constraints
        dt: time step (float)
        L: wheelbase (float)
        weights: dict with cost weights
        options: dict with optimization parameters:
            - max_admm_iter: max ADMM iterations
            - sigma: ADMM penalty parameter
            - tol_admm: ADMM convergence tolerance
            - max_ilqr_iter: max iLQR iterations per ADMM step
            - ilqr_tol: iLQR convergence tolerance

    Returns:
        X_opt: optimal state trajectory (4 x N+1 numpy array)
        U_opt: optimal control trajectory (2 x N numpy array)
        debug_info: dict with debug information
    """
    # Extract parameters
    max_admm_iter = options['max_admm_iter']
    sigma = options['sigma']
    tol_admm = options['tol_admm']
    max_ilqr_iter = options['max_ilqr_iter']
    ilqr_tol = options['ilqr_tol']

    # Initialize from candidate
    x_ref_traj = candidate.x_ref
    N = x_ref_traj.shape[1] - 1
    nx = 4
    nu = 2

    # Initialize state and control trajectories
    # Use generate_initial_trajectory for cold start (matches MATLAB)
    acc_ref = candidate.acc if hasattr(candidate, 'acc') else None
    X, U = generate_initial_trajectory(x0, x_ref_traj, N, dt, L, constraints, acc_ref)

    # Initialize ADMM variables (matches MATLAB)
    z_u = U.copy()
    lambda_u = np.zeros_like(U)
    z_x = X[:2, :].copy()
    lambda_x = np.zeros_like(z_x)
    z_v = X[3, :].copy()  # Velocity projection
    lambda_v = np.zeros_like(z_v)  # Velocity dual variable

    # Initialize primal residual tracking
    primal_residual_history = []

    # ADMM iteration loop
    for admm_iter in range(max_admm_iter):
        # Step 1: iLQR optimization (primal update)
        # Prepare ADMM data for iLQR
        admm_data = {
            'sigma': sigma,
            'z_u': z_u,
            'lambda_u': lambda_u,
            'z_x': z_x,
            'lambda_x': lambda_x,
            'z_v': z_v,
            'lambda_v': lambda_v
        }

        # iLQR iteration loop
        ilqr_converged = False
        for ilqr_iter in range(max_ilqr_iter):
            # Backward pass
            k_list, K_list, success = backward_pass_admm(
                X, U, x_ref_traj, weights, dt, L, admm_data
            )

            if not success:
                print(f'iLQR backward pass failed at iteration {ilqr_iter}')
                break

            # Forward pass
            X_new, U_new, cost_new, cost_old = forward_pass_admm(
                X, U, k_list, K_list, x_ref_traj, weights, dt, L, admm_data
            )

            # Check convergence
            cost_change = abs(cost_new - cost_old)
            if cost_change < ilqr_tol:
                ilqr_converged = True
                X = X_new
                U = U_new
                break
            else:
                X = X_new
                U = U_new

        # Step 2: Constraint projection (dual update)
        proj_result = project_constraints(U, X, lambda_u, lambda_x, sigma, constraints)
        if len(proj_result) == 3:
            z_u_new, z_x_new, z_v_new = proj_result
        else:
            z_u_new, z_x_new = proj_result
            z_v_new = X[3, :].copy()  # No velocity constraint

        # Step 3: Dual variable update
        lambda_u = lambda_u + sigma * (U - z_u_new)
        lambda_x = lambda_x + sigma * (X[:2, :] - z_x_new)
        lambda_v = lambda_v + sigma * (X[3, :] - z_v_new)

        # Update projected variables
        z_u = z_u_new
        z_x = z_x_new
        z_v = z_v_new

        # Compute primal residual for convergence check
        primal_residual = np.max([
            np.max(np.abs(U - z_u)),
            np.max(np.abs(X[:2, :] - z_x)),
            np.max(np.abs(X[3, :] - z_v))
        ])
        primal_residual_history.append(primal_residual)

        # Check ADMM convergence
        if primal_residual < tol_admm:
            print(f'ADMM converged at iteration {admm_iter}')
            break

    # Prepare debug info
    debug_info = {
        'cost_history': [],  # Would need to track costs during iLQR
        'primal_residual_history': primal_residual_history,
        'admm_iterations': admm_iter + 1
    }

    return X, U, debug_info