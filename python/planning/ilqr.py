"""
iLQR Solver - Forward and Backward Pass
Ported from MATLAB forward_pass_admm.m, backward_pass_admm.m, calculate_total_cost_admm.m
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from core.vehicle_model import update_state, get_A_matrix, get_B_matrix
from core.cost import get_cost_and_derivatives


def calculate_total_cost_admm(X, U, x_ref_traj, weights, admm_data):
    """
    Calculate total cost of trajectory with ADMM augmentation

    Args:
        X: state trajectory (4 x N+1 numpy array)
        U: control trajectory (2 x N numpy array)
        x_ref_traj: reference trajectory (4 x N+1 numpy array)
        weights: dict with cost weights
        admm_data: dict with ADMM variables

    Returns:
        total_cost: scalar total cost
    """
    N = U.shape[1]
    total_cost = 0

    for i in range(N):
        step_admm = {'sigma': admm_data['sigma']}
        if 'z_u' in admm_data and admm_data['z_u'] is not None:
            step_admm['z_u'] = admm_data['z_u'][:, i]
            step_admm['lambda_u'] = admm_data['lambda_u'][:, i]
        if 'z_x' in admm_data and admm_data['z_x'] is not None:
            step_admm['z_x'] = admm_data['z_x'][:, i]
            step_admm['lambda_x'] = admm_data['lambda_x'][:, i]
        if 'z_v' in admm_data and admm_data['z_v'] is not None:
            step_admm['z_v'] = admm_data['z_v'][i]
            step_admm['lambda_v'] = admm_data['lambda_v'][i]

        if i == 0:
            u_prev = np.zeros(2)
        else:
            u_prev = U[:, i-1]

        c, _, _, _, _, _ = get_cost_and_derivatives(
            X[:, i], U[:, i], u_prev, x_ref_traj[:, i], weights, step_admm, False
        )
        total_cost += c

    # Terminal cost
    step_admm_end = {'sigma': admm_data['sigma']}
    if 'z_x' in admm_data and admm_data['z_x'] is not None:
        step_admm_end['z_x'] = admm_data['z_x'][:, N]
        step_admm_end['lambda_x'] = admm_data['lambda_x'][:, N]
    if 'z_v' in admm_data and admm_data['z_v'] is not None:
        step_admm_end['z_v'] = admm_data['z_v'][N]
        step_admm_end['lambda_v'] = admm_data['lambda_v'][N]

    c, _, _, _, _, _ = get_cost_and_derivatives(
        X[:, N], np.zeros(2), np.zeros(2), x_ref_traj[:, N], weights, step_admm_end, True
    )
    total_cost += c

    return total_cost


def forward_pass_admm(X_old, U_old, k_list, K_list, x_ref_traj, weights, dt, L, admm_data):
    """
    Forward pass with line search

    Args:
        X_old: previous state trajectory (4 x N+1 numpy array)
        U_old: previous control trajectory (2 x N numpy array)
        k_list: feedforward gains (2 x N numpy array)
        K_list: feedback gains (2 x 4 x N numpy array)
        x_ref_traj: reference trajectory (4 x N+1 numpy array)
        weights: dict with cost weights
        dt: time step (float)
        L: wheelbase (float)
        admm_data: dict with ADMM variables

    Returns:
        X_new: new state trajectory (4 x N+1 numpy array)
        U_new: new control trajectory (2 x N numpy array)
        cost_new: new total cost (float)
        cost_old: old total cost (float)
    """
    N = U_old.shape[1]
    nx = 4
    nu = 2

    # Line search parameters
    alpha = 1.0
    min_alpha = 1e-1

    # Compute cost of the old trajectory
    cost_old = calculate_total_cost_admm(X_old, U_old, x_ref_traj, weights, admm_data)

    while alpha > min_alpha:
        X_new = np.zeros((nx, N + 1))
        U_new = np.zeros((nu, N))
        X_new[:, 0] = X_old[:, 0]  # Keep initial state

        # Simulate new trajectory
        for i in range(N):
            # Compute control law (eq.17)
            delta_x = X_new[:, i] - X_old[:, i]
            delta_u = alpha * k_list[:, i] + K_list[:, :, i] @ delta_x
            U_new[:, i] = U_old[:, i] + delta_u

            # Apply control and propagate state
            X_new[:, i+1] = update_state(X_new[:, i], U_new[:, i], dt, L)

        cost_new = calculate_total_cost_admm(X_new, U_new, x_ref_traj, weights, admm_data)

        # Accept new trajectory if cost decreased (allow 5% degradation to escape local minima)
        if cost_new < cost_old * 1.05:
            return X_new, U_new, cost_new, cost_old
        else:
            # Otherwise reduce alpha and retry
            alpha = alpha * 0.5

    # If line search fails, rescue measure: accept if cost increase is within numerical noise
    if cost_new - cost_old < 1e-3:
        # Slightly worse trajectory accepted to avoid stagnation
        return X_new, U_new, cost_new, cost_old
    else:
        # Failed: rollback to old trajectory
        return X_old, U_old, cost_old, cost_old


def backward_pass_admm(X, U, x_ref_traj, weights, dt, L, admm_data):
    """
    Backward pass with Riccati recursion and regularization

    Args:
        X: state trajectory (4 x N+1 numpy array)
        U: control trajectory (2 x N numpy array)
        x_ref_traj: reference trajectory (4 x N+1 numpy array)
        weights: dict with cost weights
        dt: time step (float)
        L: wheelbase (float)
        admm_data: dict with ADMM variables

    Returns:
        k_list: feedforward gains (2 x N numpy array)
        K_list: feedback gains (2 x 4 x N numpy array)
        success: boolean indicating if backward pass succeeded
    """
    N = U.shape[1]
    nx = 4
    nu = 2

    k_list = np.zeros((nu, N))
    K_list = np.zeros((nu, nx, N))

    # 1. Terminal cost
    step_admm = {'sigma': admm_data['sigma']}
    if 'z_x' in admm_data and admm_data['z_x'] is not None:
        step_admm['z_x'] = admm_data['z_x'][:, N]
        step_admm['lambda_x'] = admm_data['lambda_x'][:, N]
    if 'z_v' in admm_data and admm_data['z_v'] is not None:
        step_admm['z_v'] = admm_data['z_v'][N]
        step_admm['lambda_v'] = admm_data['lambda_v'][N]

    _, Vx, _, Vxx, _, _ = get_cost_and_derivatives(
        X[:, N], np.zeros(nu), np.zeros(nu), x_ref_traj[:, N], weights, step_admm, True
    )
    success = True

    # Regularization parameters initialization
    reg_mu = 1e-6      # Initial regularization
    reg_min = 1e-6     # Minimum regularization
    reg_max = 1e10     # Maximum regularization (failure if exceeded)
    reg_scale = 10     # Multiplier for increasing reg_mu

    # 2. Backward pass
    for i in range(N - 1, -1, -1):
        x = X[:, i]
        u = U[:, i]
        x_ref = x_ref_traj[:, i]

        if i == 0:
            u_prev = np.zeros(2)
        else:
            u_prev = U[:, i-1]

        step_admm = {'sigma': admm_data['sigma']}
        # Extract control constraint data for step i
        if 'z_u' in admm_data and admm_data['z_u'] is not None:
            step_admm['z_u'] = admm_data['z_u'][:, i]
            step_admm['lambda_u'] = admm_data['lambda_u'][:, i]
        # Extract state constraint data for step i
        if 'z_x' in admm_data and admm_data['z_x'] is not None:
            step_admm['z_x'] = admm_data['z_x'][:, i]
            step_admm['lambda_x'] = admm_data['lambda_x'][:, i]
        # Extract velocity constraint data for step i
        if 'z_v' in admm_data and admm_data['z_v'] is not None:
            step_admm['z_v'] = admm_data['z_v'][i]
            step_admm['lambda_v'] = admm_data['lambda_v'][i]

        # Dynamics Jacobians
        A = get_A_matrix(x, u, dt, L)
        B = get_B_matrix(x, u, dt, L)

        # Cost derivatives
        _, lx, lu, lxx, luu, lux = get_cost_and_derivatives(
            x, u, u_prev, x_ref, weights, step_admm, False
        )

        # Compute Q-function derivatives (eq.14)
        Qx = lx + A.T @ Vx
        Qu = lu + B.T @ Vx
        Qxx = lxx + A.T @ Vxx @ A
        Qux = lux + B.T @ Vxx @ A
        Quu = luu + B.T @ Vxx @ B

        # Regularize Quu for numerical stability
        # Find mu such that (Quu + mu*I) is positive definite
        is_spd = False
        Quu_reg = Quu

        # Try increasing regularization until PD or reach limit
        while not is_spd:
            Quu_reg = Quu + reg_mu * np.eye(nu)

            # Check positive-definiteness via Cholesky: p==0 means PD
            try:
                R_chol = np.linalg.cholesky(Quu_reg)
                is_spd = True
            except np.linalg.LinAlgError:
                # Matrix is not positive definite, increase regularization
                print(f'Regularizing Quu at step {i} with mu = {reg_mu:.2e}')
                reg_mu = reg_mu * reg_scale

                # Abort if exceeded maximum allowed regularization
                if reg_mu > reg_max:
                    success = False
                    print(f'Backward pass failed at step {i}: Quu regularization exceeded max limit.')
                    return k_list, K_list, success

        # Compute control gains using Cholesky factor for numerical stability
        # Quu_reg * k = -Qu  =>  R'*R*k = -Qu  => k = -R\(R'\Qu)
        k = -np.linalg.solve(R_chol, np.linalg.solve(R_chol.T, Qu))
        K = -np.linalg.solve(R_chol, np.linalg.solve(R_chol.T, Qux))

        k_list[:, i] = k
        K_list[:, :, i] = K

        # Update V derivatives (eq.18)
        Vx = Qx - K.T @ Quu_reg @ k
        Vxx = Qxx - K.T @ Quu_reg @ K

        # Enforce symmetry (numeric errors may break symmetry)
        Vxx = 0.5 * (Vxx + Vxx.T)

        # If regularization succeeded at this step, try reducing reg_mu for next step
        reg_mu = max(reg_min, reg_mu / reg_scale)

    return k_list, K_list, success