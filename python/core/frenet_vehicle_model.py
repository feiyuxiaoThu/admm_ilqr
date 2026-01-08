"""
Frenet Coordinate Vehicle Model
Represents vehicle dynamics in Frenet coordinates (s, d) for curved road optimization
"""
import numpy as np


def update_state_frenet(x_frenet, u, dt, kappa_s):
    """
    Update state in Frenet coordinates

    State: [s, d, theta_d, v_s] (4x1)
    Control: [a_s, v_d] (2x1)
    - s: longitudinal position along reference line
    - d: lateral offset from reference line
    - theta_d: heading angle error (theta_global - theta_ref)
    - v_s: longitudinal velocity
    - a_s: longitudinal acceleration
    - v_d: lateral velocity

    Args:
        x_frenet: current Frenet state [s, d, theta_d, v_s] (4x1 numpy array)
        u: control input [a_s, v_d] (2x1 numpy array)
        dt: time step (float)
        kappa_s: curvature at current s position (float)

    Returns:
        x_next: next Frenet state (4x1 numpy array)
    """
    s = x_frenet[0]
    d = x_frenet[1]
    theta_d = x_frenet[2]
    v_s = x_frenet[3]

    a_s = u[0]
    v_d = u[1]

    # Frenet kinematics
    # ds/dt = v_s / (1 - kappa * d)
    # dd/dt = v_d
    # d(theta_d)/dt = v_d * kappa
    # dv_s/dt = a_s

    ds_dot = v_s / (1 - kappa_s * d + 1e-10)
    dd_dot = v_d
    dtheta_d_dot = v_d * kappa_s
    dv_s_dot = a_s

    # Euler integration
    s_next = s + ds_dot * dt
    d_next = d + dd_dot * dt
    theta_d_next = theta_d + dtheta_d_dot * dt
    v_s_next = v_s + dv_s_dot * dt

    x_next = np.array([s_next, d_next, theta_d_next, v_s_next])

    return x_next


def get_A_matrix_frenet(x_frenet, kappa_s, dkappa_ds):
    """
    Get state transition matrix A in Frenet coordinates

    dx/dt = A * x + B * u

    Args:
        x_frenet: current Frenet state [s, d, theta_d, v_s] (4x1 numpy array)
        kappa_s: curvature at current s position (float)
        dkappa_ds: derivative of curvature w.r.t. s (float)

    Returns:
        A: state transition matrix (4x4 numpy array)
    """
    s = x_frenet[0]
    d = x_frenet[1]
    v_s = x_frenet[3]

    # Partial derivatives
    denom = (1 - kappa_s * d + 1e-10)**2

    # A matrix
    A = np.zeros((4, 4))

    # ds/dt = v_s / (1 - kappa * d)
    A[0, 3] = 1 / (1 - kappa_s * d + 1e-10)  # ds/dt w.r.t. v_s
    A[0, 1] = v_s * kappa_s / denom  # ds/dt w.r.t. d (if kappa varies with d)

    # dd/dt = v_d (no state dependence)
    A[1, :] = 0

    # d(theta_d)/dt = v_d * kappa
    # If kappa varies with s, we need dkappa/ds
    A[2, 0] = v_s * dkappa_ds  # d(theta_d)/dt w.r.t. s (through kappa variation)

    # dv_s/dt = a_s (no state dependence)
    A[3, :] = 0

    return A


def get_B_matrix_frenet(x_frenet, kappa_s):
    """
    Get control input matrix B in Frenet coordinates

    dx/dt = A * x + B * u

    Args:
        x_frenet: current Frenet state [s, d, theta_d, v_s] (4x1 numpy array)
        kappa_s: curvature at current s position (float)

    Returns:
        B: control input matrix (4x2 numpy array)
    """
    B = np.zeros((4, 2))

    # ds/dt = v_s / (1 - kappa * d) (no direct control input)
    B[0, :] = 0

    # dd/dt = v_d
    B[1, 1] = 1  # dd/dt w.r.t. v_d

    # d(theta_d)/dt = v_d * kappa
    B[2, 1] = kappa_s  # d(theta_d)/dt w.r.t. v_d

    # dv_s/dt = a_s
    B[3, 0] = 1  # dv_s/dt w.r.t. a_s

    return B


def frenet_to_cartesian(x_frenet, ref_line):
    """
    Convert Frenet state to Cartesian state

    Args:
        x_frenet: Frenet state [s, d, theta_d, v_s] (4x1 numpy array)
        ref_line: ReferenceLine object

    Returns:
        x_cart: Cartesian state [X, Y, theta, v] (4x1 numpy array)
    """
    s = x_frenet[0]
    d = x_frenet[1]
    theta_d = x_frenet[2]
    v_s = x_frenet[3]

    # Get reference state at s
    x_ref, y_ref, theta_ref, kappa_ref = ref_line.get_state_at_s(s)

    # Convert to Cartesian
    # X = X_ref - d * sin(theta_ref)
    # Y = Y_ref + d * cos(theta_ref)
    # theta = theta_ref + theta_d
    # v = v_s / (1 - kappa_ref * d)

    x = x_ref - d * np.sin(theta_ref)
    y = y_ref + d * np.cos(theta_ref)
    theta = theta_ref + theta_d
    v = v_s / (1 - kappa_ref * d + 1e-10)

    x_cart = np.array([x, y, theta, v])

    return x_cart


def cartesian_to_frenet(x_cart, ref_line):
    """
    Convert Cartesian state to Frenet state

    Args:
        x_cart: Cartesian state [X, Y, theta, v] (4x1 numpy array)
        ref_line: ReferenceLine object

    Returns:
        x_frenet: Frenet state [s, d, theta_d, v_s] (4x1 numpy array)
    """
    x = x_cart[0]
    y = x_cart[1]
    theta = x_cart[2]
    v = x_cart[3]

    # Project point to reference line
    s, d = ref_line.project_point(x, y)

    # Get reference state at s
    x_ref, y_ref, theta_ref, kappa_ref = ref_line.get_state_at_s(s)

    # Convert to Frenet
    # theta_d = theta - theta_ref
    # v_s = v * (1 - kappa_ref * d)

    theta_d = theta - theta_ref
    v_s = v * (1 - kappa_ref * d)

    x_frenet = np.array([s, d, theta_d, v_s])

    return x_frenet


def frenet_control_to_cartesian(u_frenet, x_frenet, ref_line):
    """
    Convert Frenet control to Cartesian control

    Args:
        u_frenet: Frenet control [a_s, v_d] (2x1 numpy array)
        x_frenet: Frenet state [s, d, theta_d, v_s] (4x1 numpy array)
        ref_line: ReferenceLine object

    Returns:
        u_cart: Cartesian control [a, delta] (2x1 numpy array)
    """
    a_s = u_frenet[0]
    v_d = u_frenet[1]

    s = x_frenet[0]
    d = x_frenet[1]
    v_s = x_frenet[3]

    # Get reference state at s
    x_ref, y_ref, theta_ref, kappa_ref = ref_line.get_state_at_s(s)

    # Convert to Cartesian control
    # a = a_s (approximation, ignoring curvature effects)
    # delta = arctan(L * (v_d * kappa_ref + v_s * dkappa/ds)) / v_s

    # Simplified: use lateral velocity to estimate steering
    # This is an approximation; for exact conversion, need more complex kinematics

    a = a_s

    # Steering angle from lateral dynamics
    # delta = arctan(L * (v_d + v_s * kappa_ref * d) / v_s)
    # Simplified lateral control
    if abs(v_s) < 1e-3:
        delta = 0
    else:
        # Lateral acceleration in Frenet frame
        a_lat = v_d * kappa_ref + v_s**2 * kappa_ref / (1 - kappa_ref * d + 1e-10)
        delta = np.arctan(a_lat / v_s**2)  # Simplified

    u_cart = np.array([a, delta])

    return u_cart