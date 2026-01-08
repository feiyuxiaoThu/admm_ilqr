"""
Vehicle Dynamics - Kinematic Bicycle Model
Ported from MATLAB update_state.m, get_A_matrix.m, get_B_matrix.m
"""
import numpy as np


def update_state(x, u, dt, L):
    """
    Integrate vehicle kinematics using forward Euler

    Args:
        x: state vector [X; Y; phi; v] (4x1 numpy array)
        u: control vector [a; delta] (2x1 numpy array)
        dt: time step (float)
        L: wheelbase (float)

    Returns:
        x_next: state at next time step (4x1 numpy array)

    Note: 0-based indexing (Python) vs 1-based (MATLAB)
    """
    phi = x[2]  # heading angle
    v = x[3]    # velocity

    a = u[0]    # acceleration
    delta = u[1]  # steering angle

    # state derivative (dx/dt)
    x_dot = np.array([
        v * np.cos(phi),
        v * np.sin(phi),
        v / L * np.tan(delta),
        a
    ])

    # forward Euler integration
    x_next = x + dt * x_dot
    return x_next


def get_A_matrix(x, u, dt, L):
    """
    Compute discrete-time state Jacobian A = df/dx

    Args:
        x: state vector [X; Y; phi; v] (4x1 numpy array)
        u: control vector [a; delta] (2x1 numpy array)
        dt: time step (float)
        L: wheelbase (float)

    Returns:
        A: 4x4 state Jacobian (numpy array)

    Note: 0-based indexing (Python) vs 1-based (MATLAB)
    """
    phi = x[2]
    v = x[3]
    delta = u[1]

    # build Jacobian A
    A = np.eye(4)
    A[0, 2] = -v * np.sin(phi) * dt
    A[0, 3] = np.cos(phi) * dt
    A[1, 2] = v * np.cos(phi) * dt
    A[1, 3] = np.sin(phi) * dt
    A[2, 3] = np.tan(delta) / L * dt

    # Example form:
    # A = [1, 0, -v*sin(phi)*dt, cos(phi)*dt;
    #      0, 1,  v*cos(phi)*dt, sin(phi)*dt;
    #      0, 0,  1,             tan(delta)/L*dt;
    #      0, 0,  0,             1]
    return A


def get_B_matrix(x, u, dt, L):
    """
    Compute discrete-time control Jacobian B = df/du

    Args:
        x: state vector [X; Y; phi; v] (4x1 numpy array)
        u: control vector [a; delta] (2x1 numpy array)
        dt: time step (float)
        L: wheelbase (float)

    Returns:
        B: 4x2 control Jacobian (numpy array)

    Note: 0-based indexing (Python) vs 1-based (MATLAB)
    """
    v = x[3]
    delta = u[1]

    # build Jacobian B
    B = np.zeros((4, 2))
    B[2, 1] = v / (L * np.cos(delta)**2) * dt
    B[3, 0] = dt

    # B = [0, 0;
    #      0, 0;
    #      0, v / (L * cos(delta)^2) * dt;
    #      dt, 0]
    return B