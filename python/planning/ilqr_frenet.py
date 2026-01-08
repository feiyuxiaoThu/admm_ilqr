"""
iLQR Solver - Forward and Backward Pass in Frenet Coordinates
Optimization is performed in Frenet (s, d) coordinates for curved road support
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from core.frenet_vehicle_model import (
    update_state_frenet,
    get_A_matrix_frenet,
    get_B_matrix_frenet,
    frenet_to_cartesian,
    cartesian_to_frenet
)
from core.cost import get_cost_and_derivatives


def calculate_total_cost_admm_frenet(X_frenet, U_frenet, x_ref_frenet, weights, admm_data):
    """
    Calculate total cost of trajectory in Frenet coordinates with ADMM augmentation

    Args:
        X_frenet: Frenet state trajectory (4 x N+1 numpy array)
        U_frenet: Frenet control trajectory (2 x N numpy array)
        x_ref_frenet: Frenet reference trajectory (4 x N+1 numpy array)
        weights: dict with cost weights
        admm_data: dict with ADMM variables

    Returns:
        total_cost: scalar total cost
    """
    N = U_frenet.shape[1]
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
            u_prev = U_frenet[:, i-1]

        c, _, _, _, _, _ = get_cost_and_derivatives(
            X_frenet[:, i], U_frenet[:, i], u_prev, x_ref_frenet[:, i], weights, step_admm, False
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
        X_frenet[:, N], np.zeros(2), np.zeros(2), x_ref_frenet[:, N], weights, step_admm_end, True
    )
    total_cost += c

    return total_cost


def forward_pass_admm_frenet(X_frenet_old, U_frenet_old, k_list, K_list,
                              x_ref_frenet, weights, dt, kappa_array, dkappa_ds_array, admm_data):
    """
    Forward pass with line search in Frenet coordinates

    Args:
        X_frenet_old: previous Frenet state trajectory (4 x N+1 numpy array)
        U_frenet_old: previous Frenet control trajectory (2 x N numpy array)
        k_list: feedforward gains (2 x N numpy array)
        K_list: feedback gains (2 x 4 x N numpy array)
        x_ref_frenet: Frenet reference trajectory (4 x N+1 numpy array)
        weights: dict with cost weights
        dt: time step (float)
        kappa_array: curvature at each time step (N+1 numpy array)
        dkappa_ds_array: curvature derivative at each time step (N+1 numpy array)
        admm_data: dict with ADMM variables

    Returns:
        X_frenet_new: new Frenet state trajectory (4 x N+1 numpy array)
        U_frenet_new: new Frenet control trajectory (2 x N numpy array)
        cost_new: new total cost (float)
        cost_old: old total cost (float)
    """
    N = U_frenet_old.shape[1]
    nx = 4
    nu = 2

    # Line search parameters
    alpha = 1.0
    min_alpha = 1e-1

    # Compute cost of the old trajectory
    cost_old = calculate_total_cost_admm_frenet(X_frenet_old, U_frenet_old, x_ref_frenet, weights, admm_data)

    # Line search
    while alpha >= min_alpha:
        X_frenet_new = np.zeros((nx, N + 1))
        U_frenet_new = np.zeros((nu, N))

        X_frenet_new[:, 0] = X_frenet_old[:, 0]

        for i in range(N):
            # Compute control with feedback
            delta_x = X_frenet_new[:, i] - X_frenet_old[:, i]
            U_frenet_new[:, i] = U_frenet_old[:, i] + alpha * k_list[:, i] + K_list[:, :, i] @ delta_x

            # Update state
            X_frenet_new[:, i + 1] = update_state_frenet(
                X_frenet_new[:, i], U_frenet_new[:, i], dt, kappa_array[i]
            )

        # Compute new cost
        cost_new = calculate_total_cost_admm_frenet(X_frenet_new, U_frenet_new, x_ref_frenet, weights, admm_data)

        # Check if cost decreased
        if cost_new < cost_old:
            break
        else:
            alpha *= 0.5

    if alpha < min_alpha:
        # Line search failed, use old trajectory
        X_frenet_new = X_frenet_old.copy()
        U_frenet_new = U_frenet_old.copy()
        cost_new = cost_old

    return X_frenet_new, U_frenet_new, cost_new, cost_old


def backward_pass_admm_frenet(X_frenet, U_frenet, x_ref_frenet, weights, dt,
                               kappa_array, dkappa_ds_array, admm_data, reg_init=1e-6, reg_max=1e10):
    """
    Backward pass in Frenet coordinates

    Args:
        X_frenet: Frenet state trajectory (4 x N+1 numpy array)
        U_frenet: Frenet control trajectory (2 x N numpy array)
        x_ref_frenet: Frenet reference trajectory (4 x N+1 numpy array)
        weights: dict with cost weights
        dt: time step (float)
        kappa_array: curvature at each time step (N+1 numpy array)
        dkappa_ds_array: curvature derivative at each time step (N+1 numpy array)
        admm_data: dict with ADMM variables
        reg_init: initial regularization (float)
        reg_max: maximum regularization (float)

    Returns:
        k_list: feedforward gains (2 x N numpy array)
        K_list: feedback gains (2 x 4 x N numpy array)
        V_x: value function gradient (4 x N+1 numpy array)
        V_xx: value function Hessian (4 x 4 x N+1 numpy array)
        debug_info: dict with debug information
    """
    N = U_frenet.shape[1]
    nx = 4
    nu = 2

    k_list = np.zeros((nu, N))
    K_list = np.zeros((nu, nx, N))
    V_x = np.zeros((nx, N + 1))
    V_xx = np.zeros((nx, nx, N + 1))

    # Terminal value function
    step_admm_end = {'sigma': admm_data['sigma']}
    if 'z_x' in admm_data and admm_data['z_x'] is not None:
        step_admm_end['z_x'] = admm_data['z_x'][:, N]
        step_admm_end['lambda_x'] = admm_data['lambda_x'][:, N]
    if 'z_v' in admm_data and admm_data['z_v'] is not None:
        step_admm_end['z_v'] = admm_data['z_v'][N]
        step_admm_end['lambda_v'] = admm_data['lambda_v'][N]

    _, lx, lu, lxx, luu, lux = get_cost_and_derivatives(
        X_frenet[:, N], np.zeros(2), np.zeros(2), x_ref_frenet[:, N], weights, step_admm_end, True
    )

    V_x[:, N] = lx
    V_xx[:, :, N] = lxx

    # Backward pass
    reg = reg_init
    failed = False

    for i in range(N - 1, -1, -1):
        if i == 0:
            u_prev = np.zeros(2)
        else:
            u_prev = U_frenet[:, i-1]

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

        # Cost derivatives
        c, lx, lu, lxx, luu, lux = get_cost_and_derivatives(
            X_frenet[:, i], U_frenet[:, i], u_prev, x_ref_frenet[:, i], weights, step_admm, False
        )

        # Dynamics matrices
        A = get_A_matrix_frenet(X_frenet[:, i], kappa_array[i], dkappa_ds_array[i])
        B = get_B_matrix_frenet(X_frenet[:, i], kappa_array[i])

        # Value function derivatives
        Qx = lx + A.T @ V_x[:, i + 1]
        Qu = lu + B.T @ V_x[:, i + 1]
        Qxx = lxx + A.T @ V_xx[:, :, i + 1] @ A
        Qux = lux + B.T @ V_xx[:, :, i + 1] @ A
        Quu = luu + B.T @ V_xx[:, :, i + 1] @ B

        # Regularization
        Quu_reg = Quu + reg * np.eye(nu)

        # Cholesky decomposition
        try:
            L_mat = np.linalg.cholesky(Quu_reg)
        except np.linalg.LinAlgError:
            # Increase regularization
            reg *= 10
            if reg > reg_max:
                failed = True
                break
            Quu_reg = Quu + reg * np.eye(nu)
            L_mat = np.linalg.cholesky(Quu_reg)

        # Compute gains
        K = -np.linalg.solve(L_mat.T, np.linalg.solve(L_mat, Qux))
        k = -np.linalg.solve(L_mat.T, np.linalg.solve(L_mat, Qu))

        k_list[:, i] = k
        K_list[:, :, i] = K

        # Update value function
        V_x[:, i] = Qx + K.T @ Quu @ k + K.T @ Qu + Qux.T @ k
        V_xx[:, :, i] = Qxx + K.T @ Quu @ K + K.T @ Quu @ K + Qux.T @ K + K.T @ Qux

        # Ensure symmetry
        V_xx[:, :, i] = 0.5 * (V_xx[:, :, i] + V_xx[:, :, i].T)

    debug_info = {
        'failed': failed,
        'final_reg': reg
    }

    return k_list, K_list, V_x, V_xx, debug_info


def run_ilqr_frenet(x0_frenet, x_ref_frenet, weights, dt, kappa_array, dkappa_ds_array,
                    admm_data, max_iter=50, tol=1e-1):
    """
    Run iLQR optimization in Frenet coordinates

    Args:
        x0_frenet: initial Frenet state [s, d, theta_d, v_s] (4x1 numpy array)
        x_ref_frenet: Frenet reference trajectory (4 x N+1 numpy array)
        weights: dict with cost weights
        dt: time step (float)
        kappa_array: curvature at each time step (N+1 numpy array)
        dkappa_ds_array: curvature derivative at each time step (N+1 numpy array)
        admm_data: dict with ADMM variables
        max_iter: maximum iterations (int)
        tol: convergence tolerance (float)

    Returns:
        X_frenet_opt: optimal Frenet state trajectory (4 x N+1 numpy array)
        U_frenet_opt: optimal Frenet control trajectory (2 x N numpy array)
        debug_info: dict with debug information
    """
    N = x_ref_frenet.shape[1] - 1
    nx = 4
    nu = 2

    # Initialize trajectory with reference
    X_frenet = x_ref_frenet.copy()
    X_frenet[:, 0] = x0_frenet

    # Initialize control with zeros
    U_frenet = np.zeros((nu, N))

    cost_history = []

    for iter in range(max_iter):
        # Backward pass
        k_list, K_list, V_x, V_xx, debug_backward = backward_pass_admm_frenet(
            X_frenet, U_frenet, x_ref_frenet, weights, dt,
            kappa_array, dkappa_ds_array, admm_data
        )

        if debug_backward['failed']:
            debug_info = {
                'converged': False,
                'iterations': iter,
                'cost_history': cost_history,
                'failed': True,
                'reason': 'Backward pass failed'
            }
            return X_frenet, U_frenet, debug_info

        # Forward pass
        X_frenet_new, U_frenet_new, cost_new, cost_old = forward_pass_admm_frenet(
            X_frenet, U_frenet, k_list, K_list, x_ref_frenet, weights, dt,
            kappa_array, dkappa_ds_array, admm_data
        )

        cost_history.append(cost_new)

        # Check convergence
        if abs(cost_old - cost_new) < tol:
            X_frenet = X_frenet_new
            U_frenet = U_frenet_new
            debug_info = {
                'converged': True,
                'iterations': iter + 1,
                'cost_history': cost_history
            }
            break

        X_frenet = X_frenet_new
        U_frenet = U_frenet_new
    else:
        debug_info = {
            'converged': False,
            'iterations': max_iter,
            'cost_history': cost_history
        }

    return X_frenet, U_frenet, debug_info