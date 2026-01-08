"""
Scenario Generation - Road geometry and obstacles
Ported from MATLAB build_simulation_scenario.m, generate_obstacles.m
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry.reference_line import ReferenceLineSystem


class Obstacle:
    """Represents an obstacle as an ellipse"""

    def __init__(self, x, y, length, width, theta, vx=0.0, vy=0.0):
        """
        Initialize obstacle

        Args:
            x: x-coordinate of center (float)
            y: y-coordinate of center (float)
            length: total length (float)
            width: total width (float)
            theta: orientation angle in radians (float)
            vx: velocity in x-direction (float, default 0)
            vy: velocity in y-direction (float, default 0)
        """
        self.x = x
        self.y = y
        self.length = length  # total length
        self.width = width    # total width
        self.a = length / 2.0  # semi-major axis
        self.b = width / 2.0   # semi-minor axis
        self.theta = theta
        self.vx = vx
        self.vy = vy


class Scenario:
    """
    Represents the driving scenario with road geometry and obstacles
    """

    def __init__(self, road_type='straight', road_length=100, lane_width=4.0, num_lanes=3):
        """
        Initialize scenario

        Args:
            road_type: type of road geometry ('straight', 'sine', 's_turn')
            road_length: length of the road (float)
            lane_width: width of each lane (float)
            num_lanes: number of lanes (int)
        """
        self.road_type = road_type
        self.road_length = road_length
        self.lane_width = lane_width
        self.num_lanes = num_lanes

        # Use ReferenceLineSystem to manage lanes
        self.ref_line_system = ReferenceLineSystem(
            road_type=road_type,
            road_length=road_length,
            lane_width=lane_width,
            num_lanes=num_lanes
        )

        # Ego vehicle dimensions
        self.ego_size = {'length': 4.5, 'width': 2.0}

        # Desired velocity
        self.v_desired = 20.0

    @property
    def lanes(self):
        """Get all lanes from reference line system"""
        return self.ref_line_system.get_all_lanes()

    def get_lane(self, lane_id):
        """Get lane by ID"""
        return self.ref_line_system.get_lane(lane_id)

    def get_lane_by_position(self, x, y):
        """Find the closest lane to a given position"""
        return self.ref_line_system.get_lane_by_position(x, y)

    def get_road_bounds(self):
        """
        Get road boundary constraints for vehicle centroid

        Returns:
            tuple: (y_min, y_max) - allowed y-range for vehicle centroid
        """
        # For curved roads, this is approximate
        # In practice, should use ReferenceLine.project_point to check boundaries
        margin_safety = 0.2
        ego_half_width = self.ego_size['width'] / 2.0

        # Approximate bounds based on lane centers
        y_min = -self.lane_width / 2.0 - ego_half_width - margin_safety
        y_max = self.lane_width / 2.0 + ego_half_width + margin_safety

        return y_min, y_max


def generate_obstacles(raw_obstacles, ego_size, dt, N):
    """
    Generate obstacle trajectories over time horizon

    Args:
        raw_obstacles: list of Obstacle objects at t=0
        ego_size: dict with 'length' and 'width' of ego vehicle
        dt: time step
        N: number of time steps

    Returns:
        list of Obstacle objects with prediction field
    """
    obstacles = []

    for raw in raw_obstacles:
        # Inflate obstacle using Minkowski sum approximation
        # Matches MATLAB: obs.a = (raw.length + ego_size.length) / 2.0 + safe_margin_lon
        safe_margin_lon = 3.0  # Longitudinal safety margin (m)
        safe_margin_lat = 0.8   # Lateral safety margin (m)
        a = (raw.length + ego_size['length']) / 2.0 + safe_margin_lon
        b = (raw.width + ego_size['width']) / 2.0 + safe_margin_lat

        # Trajectory prediction using constant velocity model
        if abs(raw.vx) < 1e-3 and abs(raw.vy) < 1e-3:
            # Static obstacle - no prediction needed
            obs = Obstacle(x=raw.x, y=raw.y, length=a*2, width=b*2, theta=raw.theta, vx=0.0, vy=0.0)
            obs.prediction = None
            obstacles.append(obs)
            continue

        # Time sequence: [0, dt, 2*dt, ..., (N-1)*dt]
        t_seq = np.arange(N) * dt

        # Linear kinematic model: position(t) = position_0 + velocity * t
        pred_x = raw.x + raw.vx * t_seq
        pred_y = raw.y + raw.vy * t_seq
        pred_theta = np.full(N, raw.theta)

        # Create obstacle with prediction
        obs = Obstacle(x=raw.x, y=raw.y, length=a*2, width=b*2, theta=raw.theta, vx=raw.vx, vy=raw.vy)
        obs.a = a
        obs.b = b
        obs.prediction = np.vstack([pred_x, pred_y, pred_theta])

        obstacles.append(obs)

    return obstacles


def build_simulation_scenario(dt, N, road_type='straight'):
    """
    Build a simulation scenario with road geometry and obstacles

    Args:
        dt: time step (float)
        N: number of time steps (int)
        road_type: type of road ('straight', 'sine', 's_turn')

    Returns:
        scenario: Scenario object
        constraints: dict with constraints
        x0: initial state (4x1 numpy array)
        raw_obs_list: list of initial obstacles
    """
    # Create scenario
    scenario = Scenario(road_type=road_type, road_length=100, lane_width=4.0, num_lanes=3)

    # Ego vehicle initial state (positioned in center lane)
    x0 = np.array([0.0, 0.0, 0.0, 15.0])  # [X, Y, phi, v]

    # Road boundary constraints
    y_min, y_max = scenario.get_road_bounds()
    road_bounds = {'y_min': y_min, 'y_max': y_max}

    # Generate obstacles
    # Adjust obstacle positions to avoid direct collision with reference trajectory
    obs1 = Obstacle(x=50.0, y=0.0, length=4.5, width=2.0, theta=0.0, vx=6.0, vy=0.0)  # Moved from 40 to 50
    obs2 = Obstacle(x=10.0, y=4.0, length=4.5, width=2.0, theta=0.0, vx=8.0, vy=0.0)  # Moved from 5 to 10
    obs3 = Obstacle(x=70.0, y=-4.0, length=4.5, width=2.0, theta=0.0, vx=0.0, vy=0.0)  # Moved from 60 to 70

    raw_obs_list = [obs1, obs2, obs3]
    obstacles = generate_obstacles(raw_obs_list, scenario.ego_size, dt, N)

    # Physical constraints
    u_min = np.array([-3.0, -0.5])  # [a_min, delta_min]
    u_max = np.array([2.0, 0.5])    # [a_max, delta_max]

    # Add velocity constraints
    v_min = 0.0   # Minimum velocity (no reverse)
    v_max = 25.0  # Maximum velocity

    constraints = {
        'road_bounds': road_bounds,
        'obstacles': obstacles,
        'u_min': u_min,
        'u_max': u_max,
        'v_min': v_min,
        'v_max': v_max
    }

    return scenario, constraints, x0, raw_obs_list