"""
Visualization Functions - Plotting trajectories and results
Ported from MATLAB plot_results_multimodal.m
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Ellipse


def draw_car_box(ax, x, y, phi, length, width, color, alpha=0.1):
    """
    Draw vehicle bounding box

    Args:
        ax: matplotlib axes object
        x: x-coordinate of vehicle center
        y: y-coordinate of vehicle center
        phi: heading angle in radians
        length: vehicle length
        width: vehicle width
        color: color of the box
        alpha: transparency
    """
    # Define corners in vehicle frame
    p_corners = np.array([
        [-length/2, length/2, length/2, -length/2],
        [-width/2, -width/2, width/2, width/2]
    ])

    # Rotate to global frame
    R = np.array([[np.cos(phi), -np.sin(phi)],
                  [np.sin(phi), np.cos(phi)]])
    p_global = R @ p_corners + np.array([[x], [y]])

    # Draw polygon
    poly = Polygon(p_global.T, closed=True, facecolor=color, edgecolor='k',
                   linewidth=0.5, alpha=alpha)
    ax.add_patch(poly)


def plot_results_multimodal(results, best_idx, constraints, scenario, dt, save_path='trajectory_optimization_result.png'):
    """
    Visualize multiple candidate trajectories and optimal decision

    Args:
        results: list of dict with 'X', 'U', 'cand'
        best_idx: index of optimal trajectory
        constraints: dict with 'obstacles'
        scenario: Scenario object
        dt: time step (float)
        save_path: path to save the figure (str)
    """
    if not results:
        print("No results to plot")
        return

    # Create figure
    fig = plt.figure(figsize=(16, 8))
    fig.patch.set_facecolor('white')
    fig.canvas.manager.set_window_title('Parallel Homotopic Trajectory Optimization Results')

    # Get time vectors
    N_steps = results[0]['X'].shape[1]
    t_vec = np.arange(N_steps) * dt
    t_u_vec = np.arange(N_steps - 1) * dt

    # XY trajectory subplot (main visualization area)
    ax1 = plt.subplot2grid((2, 2), (0, 0), colspan=2)
    ax1.set_aspect('equal')
    ax1.set_xlabel('Global X [m]')
    ax1.set_ylabel('Global Y [m]')
    ax1.set_title('Multi-Candidate Trajectories & Dynamic Environment', fontsize=12)
    ax1.grid(True, alpha=0.3)

    # Draw road markings and lane boundaries
    # Use scenario's reference lines if available
    if hasattr(scenario, 'ref_line_system') and scenario.ref_line_system is not None:
        # Draw curved reference lines
        lanes = scenario.ref_line_system.get_all_lanes()
        lane_width = scenario.lane_width if hasattr(scenario, 'lane_width') else 4.0
        
        # Draw lane center lines
        for i, lane in enumerate(lanes):
            waypoints = lane.reference_line.waypoints
            ax1.plot(waypoints[:, 0], waypoints[:, 1], 'k-.', linewidth=0.5, alpha=0.7)
            # Add lane label at start
            ax1.text(waypoints[0, 0], waypoints[0, 1], f'Lane {i}',
                    color=[0.5, 0.5, 0.5], fontsize=8)
        
        # Draw lane dividers between lanes
        for i in range(len(lanes) - 1):
            # Get boundary between lane i and lane i+1
            # Offset lane i by +lane_width/2
            divider = scenario.ref_line_system._offset_lane_boundary(lanes[i], lane_width/2)
            ax1.plot(divider[:, 0], divider[:, 1], 'k--', linewidth=1.0, alpha=0.7)
        
        # Draw road boundaries (offset from outermost lanes)
        if len(lanes) > 0:
            leftmost = lanes[-1]
            rightmost = lanes[0]
            
            # Get boundary waypoints
            left_boundary = scenario.ref_line_system._offset_lane_boundary(leftmost, lane_width/2)
            right_boundary = scenario.ref_line_system._offset_lane_boundary(rightmost, -lane_width/2)
            
            ax1.plot(left_boundary[:, 0], left_boundary[:, 1], 'k-', linewidth=2.0, label='Road Boundary')
            ax1.plot(right_boundary[:, 0], right_boundary[:, 1], 'k-', linewidth=2.0)
    else:
        # Fallback to straight road visualization
        lane_centers = [0.0, -4.0, 4.0]  # Center, Right, Left
        lane_width = 4.0

        y_outer_min = min(lane_centers) - lane_width / 2
        y_outer_max = max(lane_centers) + lane_width / 2

        # Road boundaries (thick solid lines)
        ax1.axhline(y=y_outer_min, color='k', linewidth=2.0, label='Road Boundary')
        ax1.axhline(y=y_outer_max, color='k', linewidth=2.0)

        # Lane dividers (dashed lines)
        for i in range(len(lane_centers) - 1):
            y_div = (lane_centers[i] + lane_centers[i+1]) / 2
            ax1.axhline(y=y_div, color='k', linestyle='--', linewidth=1.0)

        # Lane center lines (light dash-dot lines)
        for i, center in enumerate(lane_centers):
            ax1.axhline(y=center, color=[0.7, 0.7, 0.7], linestyle='-.', linewidth=0.5)
            ax1.text(results[0]['X'][0, 0] - 5, center, f'Lane {i}',
                    color=[0.5, 0.5, 0.5], fontsize=8)

    # Draw obstacles
    if 'obstacles' in constraints and constraints['obstacles']:
        theta_circle = np.linspace(0, 2*np.pi, 50)
        circle_x = np.cos(theta_circle)
        circle_y = np.sin(theta_circle)

        for i, obs in enumerate(constraints['obstacles']):  # obstacles list
            # Draw ellipse
            scale_x = obs.a * circle_x
            scale_y = obs.b * circle_y
            R = np.array([[np.cos(obs.theta), -np.sin(obs.theta)],
                          [np.sin(obs.theta), np.cos(obs.theta)]])
            rotated = R @ np.array([scale_x, scale_y])
            final_x = rotated[0, :] + obs.x
            final_y = rotated[1, :] + obs.y

            ellipse = Ellipse((obs.x, obs.y), width=obs.a*2, height=obs.b*2,
                             angle=np.degrees(obs.theta),
                             facecolor=[1, 0.8, 0.8], edgecolor='r',
                             alpha=0.5, label=f'Obs {i}' if i == 0 else '')
            ax1.add_patch(ellipse)

            # Draw car box
            draw_car_box(ax1, obs.x, obs.y, obs.theta,
                        obs.a*2, obs.b*2, [1, 0.6, 0.6], 0.1)

    # Plot all candidate trajectories (gray, non-optimal)
    for i, result in enumerate(results):
        if i == best_idx:
            continue
        X = result['X']
        ax1.plot(X[0, :], X[1, :], '-', color=[0.6, 0.6, 0.6, 0.5],
                linewidth=1.2, label='Candidates' if i == 1 else '')

    # Plot optimal trajectory (highlighted in green)
    if best_idx >= 0:
        X_best = results[best_idx]['X']
        cand_name = results[best_idx]['cand'].name.replace('_', '-')

        ax1.plot(X_best[0, :], X_best[1, :], 'g-', linewidth=2.5,
                label=f'Selected: {cand_name}')

        # Start and goal points
        ax1.plot(X_best[0, 0], X_best[1, 0], 'go', markerfacecolor='g')
        ax1.plot(X_best[0, -1], X_best[1, -1], 'gd', markerfacecolor='g')

        # Ego vehicle snapshots along trajectory
        draw_interval = max(1, N_steps // 6)
        for k in range(0, N_steps, draw_interval):
            draw_car_box(ax1, X_best[0, k], X_best[1, k], X_best[2, k],
                        scenario.ego_size['length'], scenario.ego_size['width'],
                        'g', 0.1)

        # Collect all data to determine axis ranges
    all_x = []
    all_y = []

    # Add trajectory data
    for result in results:
        X = result['X']
        all_x.extend(X[0, :])
        all_y.extend(X[1, :])

    # Add obstacle data
    if 'obstacles' in constraints and constraints['obstacles']:
        for obs in constraints['obstacles']:
            # Add obstacle center and approximate bounds
            all_x.append(obs.x)
            all_y.append(obs.y)
            # Add ellipse boundary (approximate)
            all_x.extend([obs.x - obs.a, obs.x + obs.a])
            all_y.extend([obs.y - obs.b, obs.y + obs.b])

    # Add road boundary data if available
    if hasattr(scenario, 'ref_line_system') and scenario.ref_line_system is not None:
        lanes = scenario.ref_line_system.get_all_lanes()
        lane_width = scenario.lane_width if hasattr(scenario, 'lane_width') else 4.0
        
        if len(lanes) > 0:
            leftmost = lanes[-1]
            rightmost = lanes[0]
            
            left_boundary = scenario.ref_line_system._offset_lane_boundary(leftmost, lane_width/2)
            right_boundary = scenario.ref_line_system._offset_lane_boundary(rightmost, -lane_width/2)
            
            all_x.extend(left_boundary[:, 0])
            all_y.extend(left_boundary[:, 1])
            all_x.extend(right_boundary[:, 0])
            all_y.extend(right_boundary[:, 1])

    # Calculate axis ranges with padding
    if all_x and all_y:
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        
        # Add padding (10% of range or minimum 5m)
        x_padding = max(5, (x_max - x_min) * 0.1)
        y_padding = max(5, (y_max - y_min) * 0.1)
        
        ax1.set_xlim([x_min - x_padding, x_max + x_padding])
        ax1.set_ylim([y_min - y_padding, y_max + y_padding])

    ax1.legend(loc='upper right')

    # Velocity profile subplot
    ax2 = plt.subplot(2, 2, 3)
    ax2.set_title('Velocity Profile', fontsize=11)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [m/s]')
    ax2.grid(True, alpha=0.3)

    for i, result in enumerate(results):
        if i == best_idx:
            continue
        X = result['X']
        ax2.plot(t_vec, X[3, :], '-', color=[0.7, 0.7, 0.7])

    if best_idx >= 0:
        X_best = results[best_idx]['X']
        ax2.plot(t_vec, X_best[3, :], 'g-', linewidth=2.0)
        v_target = results[best_idx]['cand'].v_target
        ax2.axhline(y=v_target, color='g', linestyle='--', alpha=0.5)

    ax2.axhline(y=scenario.v_desired, color='k', linestyle='--', label='Target')

    # Acceleration profile subplot
    ax3 = plt.subplot(2, 2, 4)
    ax3.set_title('Acceleration Profile', fontsize=11)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Acc [m/s^2]')
    ax3.grid(True, alpha=0.3)

    for i, result in enumerate(results):
        if i == best_idx:
            continue
        U = result['U']
        ax3.plot(t_u_vec, U[0, :], '-', color=[0.7, 0.7, 0.7])

    if best_idx >= 0:
        U_best = results[best_idx]['U']
        ax3.plot(t_u_vec, U_best[0, :], 'g-', linewidth=2.0)

    ax3.axhline(y=constraints['u_max'][0], color='r', linestyle=':', linewidth=1.5)
    ax3.axhline(y=constraints['u_min'][0], color='r', linestyle=':', linewidth=1.5)

    plt.tight_layout()
    # Save figure instead of showing (for non-interactive environments)
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f'Visualization saved to: {save_path}')
    plt.close()