"""
Cost Function - Stage cost with ADMM augmentation
Ported from MATLAB get_cost_and_derivatives.m
"""
import numpy as np


def get_cost_and_derivatives(x, u, u_prev, x_ref, weights, step_admm=None, is_terminal=False):
    """
    Compute stage cost and derivatives with optional ADMM augmentation

    Combines iLQR tracking cost with control rate penalty and optional ADMM penalty terms.
    Base cost: l = 0.5 * (z - z_ref)' * Q * (z - z_ref)

    Args:
        x: state vector [X; Y; phi; v] (4x1 numpy array)
        u: control vector [a; delta] (2x1 numpy array)
        u_prev: previous control [a_prev; delta_prev] (2x1 numpy array)
        x_ref: reference state [X_ref; Y_ref; phi_ref; v_ref] (4x1 numpy array)
        weights: dict with keys:
            - q_pos_x, q_pos_y, q_vel: stage weights
            - r_acc, r_steer: control weights
            - q_pos_x_term, q_pos_y_term, q_vel_term: terminal weights
            - r_delta_acc, r_delta_steer: control rate weights
        step_admm: dict with optional ADMM variables (None disables ADMM):
            - sigma: penalty parameter (float)
            - z_u: projected control [a_proj; delta_proj] (2x1 numpy array)
            - lambda_u: dual variable for control (2x1 numpy array)
            - z_x: projected position [X_proj; Y_proj] (2x1 numpy array)
            - lambda_x: dual variable for position (2x1 numpy array)
        is_terminal: boolean flag, determines if terminal or stage cost weights are used

    Returns:
        l: scalar cost (iLQR + control rate + ADMM terms if enabled)
        lx: gradient w.r.t. state x (4x1 numpy array)
        lu: gradient w.r.t. control u (2x1 numpy array)
        lxx: Hessian w.r.t. state x (4x4 numpy array)
        luu: Hessian w.r.t. control u (2x2 numpy array)
        lux: cross Hessian term (2x4 numpy array)
    """
    nx = 4
    nu = 2

    # 1. Calculate raw iLQR cost (tracking + control effort)
    # extract weights
    q_pos_x = weights['q_pos_x']
    q_pos_y = weights['q_pos_y']
    q_vel = weights['q_vel']
    r_acc = weights['r_acc']
    r_steer = weights['r_steer']
    q_pos_x_term = weights['q_pos_x_term']
    q_pos_y_term = weights['q_pos_y_term']
    q_vel_term = weights['q_vel_term']

    # full Hessian Q for [x; u]
    if not is_terminal:
        Q = np.diag([q_pos_x, q_pos_y, 0, q_vel, r_acc, r_steer])
    else:
        Q = np.diag([q_pos_x_term, q_pos_y_term, 0, q_vel_term, r_acc, r_steer])

    # reference and current stacked vectors
    u_ref = np.zeros(nu)
    z_ref = np.concatenate([x_ref, u_ref])
    z = np.concatenate([x, u])

    # error vector
    dz = z - z_ref

    # 1) scalar cost
    l = 0.5 * dz.T @ Q @ dz

    # 2) gradients
    lz = Q @ dz
    lx = lz[:nx]
    lu = lz[nx:nx+nu]

    # 3) Hessian (constant)
    lzz = Q
    lxx = lzz[:nx, :nx]
    luu = lzz[nx:nx+nu, nx:nx+nu]
    lux = lzz[nx:nx+nu, :nx]

    # 4) Add control rate penalty if not terminal
    if not is_terminal:
        r_d_acc = weights['r_delta_acc']
        r_d_steer = weights['r_delta_steer']
        R_delta = np.diag([r_d_acc, r_d_steer])

        delta_u = u - u_prev

        # 1. Add to cost
        l = l + 0.5 * delta_u.T @ R_delta @ delta_u

        # 2. Add to gradients lu
        # d/du (0.5 * (u-up)' R (u-up)) = R * (u - up)
        lu = lu + R_delta @ delta_u

        # 3. Add to Hessian luu
        # d^2/du^2 = R
        luu = luu + R_delta

    # 2. Add ADMM penalty terms to cost and derivatives (Augmented Lagrangian)
    if step_admm is not None and step_admm:
        sigma = step_admm['sigma']

        # Handle control ADMM terms (box constraints on u)
        if 'z_u' in step_admm and step_admm['z_u'] is not None:
            z_u = step_admm['z_u']  # projected control (2x1)
            lambda_u = step_admm['lambda_u']  # dual variable (2x1)

            # target value: u_target = z_u - lambda_u / sigma
            u_target = z_u - lambda_u / sigma

            # error
            diff_u = u - u_target

            # update cost: l += (sigma / 2) * ||u - u_target||^2
            l = l + (sigma / 2) * (diff_u.T @ diff_u)

            # update gradient: lu += sigma * (u - u_target)
            lu = lu + sigma * diff_u

            # update Hessian: luu += sigma * I
            luu = luu + sigma * np.eye(nu)

        # Handle position ADMM terms (constraints on X, Y only)
        # Note: we only apply constraints to the first two state dims (X, Y)
        if 'z_x' in step_admm and step_admm['z_x'] is not None:
            z_x = step_admm['z_x']  # projected position (2x1)
            lambda_x = step_admm['lambda_x']  # dual variable (2x1)

            # current position
            pos_curr = x[:2]

            # target value
            pos_target = z_x - lambda_x / sigma

            # error
            diff_pos = pos_curr - pos_target

            # update cost
            l = l + (sigma / 2) * (diff_pos.T @ diff_pos)

            # update gradient
            lx[:2] = lx[:2] + sigma * diff_pos

            # update Hessian
            lxx[:2, :2] = lxx[:2, :2] + sigma * np.eye(2)

        # Handle velocity ADMM terms (constraints on V only)
        # Note: we only apply constraints to the third state dim (V)
        if 'z_v' in step_admm and step_admm['z_v'] is not None:
            z_v = step_admm['z_v']  # projected velocity (scalar)
            lambda_v = step_admm['lambda_v']  # dual variable (scalar)

            # current velocity
            vel_curr = x[3]

            # target value
            vel_target = z_v - lambda_v / sigma

            # error
            diff_vel = vel_curr - vel_target

            # update cost
            l = l + (sigma / 2) * (diff_vel**2)

            # update gradient
            lx[3] = lx[3] + sigma * diff_vel

            # update Hessian
            lxx[3, 3] = lxx[3, 3] + sigma

    return l, lx, lu, lxx, luu, lux