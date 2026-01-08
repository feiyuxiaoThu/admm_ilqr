"""
Reference Line System - Curved Road Support
Represents lanes as geometric scatter points (polylines) instead of scalar Y-values
"""
import numpy as np


class ReferenceLine:
    """
    Represents a lane/road as a set of scatter points (Reference Line)

    Uses discrete waypoints to represent arbitrary road geometries.
    Supports conversion between global coordinates (x, y) and Frenet coordinates (s, d).
    Uses linear interpolation between waypoints for smooth queries.
    """

    def __init__(self, waypoints):
        """
        Initialize reference line from waypoints (scatter points)

        Args:
            waypoints: list of (x, y) tuples or numpy array of shape (N, 2)
                      These are the discrete scatter points defining the reference line
        """
        waypoints = np.array(waypoints)
        self.waypoints = waypoints
        self.num_points = len(waypoints)

        # Compute cumulative arc length (s-coordinate) for each waypoint
        dx = np.diff(waypoints[:, 0])
        dy = np.diff(waypoints[:, 1])
        ds = np.sqrt(dx**2 + dy**2)
        self.s = np.concatenate([[0], np.cumsum(ds)])

        # Precompute heading for each waypoint (tangent direction)
        self.theta = np.zeros(self.num_points)
        for i in range(self.num_points):
            if i == 0:
                # Use forward difference for first point
                dx = waypoints[1, 0] - waypoints[0, 0]
                dy = waypoints[1, 1] - waypoints[0, 1]
            elif i == self.num_points - 1:
                # Use backward difference for last point
                dx = waypoints[i, 0] - waypoints[i-1, 0]
                dy = waypoints[i, 1] - waypoints[i-1, 1]
            else:
                # Use central difference for interior points
                dx = waypoints[i+1, 0] - waypoints[i-1, 0]
                dy = waypoints[i+1, 1] - waypoints[i-1, 1]

            self.theta[i] = np.arctan2(dy, dx)

        # Precompute curvature for each waypoint (finite difference)
        self.kappa = np.zeros(self.num_points)
        for i in range(1, self.num_points - 1):
            # Curvature from change in heading over arc length
            d_theta = self.theta[i+1] - self.theta[i-1]
            ds_avg = (self.s[i+1] - self.s[i-1]) / 2
            self.kappa[i] = d_theta / (ds_avg + 1e-10)

        # For endpoints, use nearest neighbor curvature
        self.kappa[0] = self.kappa[1]
        self.kappa[-1] = self.kappa[-2]

        self.s_max = self.s[-1]

    def get_state_at_s(self, s):
        """
        Get global state at longitudinal distance s using linear interpolation

        Args:
            s: longitudinal distance along reference line (float or array)

        Returns:
            x, y, theta, kappa: position, heading, and curvature
                - x: x-coordinate (float or array)
                - y: y-coordinate (float or array)
                - theta: heading angle in radians (float or array)
                - kappa: curvature (float or array)
        """
        # Handle array input
        s_array = np.atleast_1d(s)
        s_clamped = np.clip(s_array, 0, self.s_max)

        # Initialize output arrays
        x_out = np.zeros_like(s_clamped)
        y_out = np.zeros_like(s_clamped)
        theta_out = np.zeros_like(s_clamped)
        kappa_out = np.zeros_like(s_clamped)

        # For each s value, find the segment and interpolate
        for i, s_val in enumerate(s_clamped):
            # Find the segment index
            if s_val >= self.s_max:
                idx = self.num_points - 2
                alpha = 1.0
            else:
                idx = np.searchsorted(self.s, s_val) - 1
                idx = max(0, min(idx, self.num_points - 2))
                alpha = (s_val - self.s[idx]) / (self.s[idx+1] - self.s[idx] + 1e-10)

            # Linear interpolation
            x_out[i] = (1 - alpha) * self.waypoints[idx, 0] + alpha * self.waypoints[idx+1, 0]
            y_out[i] = (1 - alpha) * self.waypoints[idx, 1] + alpha * self.waypoints[idx+1, 1]
            theta_out[i] = (1 - alpha) * self.theta[idx] + alpha * self.theta[idx+1]
            kappa_out[i] = (1 - alpha) * self.kappa[idx] + alpha * self.kappa[idx+1]

        # Return scalar if input was scalar
        if np.isscalar(s):
            return x_out[0], y_out[0], theta_out[0], kappa_out[0]
        else:
            return x_out, y_out, theta_out, kappa_out

    def project_point(self, x, y):
        """
        Project a global point (x, y) onto the reference line

        Finds the Frenet coordinates (s, d) of the point relative to this line
        using nearest neighbor search on the scatter points.

        Args:
            x: x-coordinate of point (float)
            y: y-coordinate of point (float)

        Returns:
            s: longitudinal distance along reference line (float)
            d: lateral offset from reference line (float, positive = left)
        """
        # Find closest waypoint by Euclidean distance
        distances = np.sqrt((self.waypoints[:, 0] - x)**2 + (self.waypoints[:, 1] - y)**2)
        min_idx = np.argmin(distances)

        # Get the closest point as reference
        x_ref = self.waypoints[min_idx, 0]
        y_ref = self.waypoints[min_idx, 1]
        theta_ref = self.theta[min_idx]

        # Compute longitudinal distance s
        s = self.s[min_idx]

        # Compute lateral offset d
        # d = (x - x_ref) * (-sin(theta)) + (y - y_ref) * cos(theta)
        d = (x - x_ref) * (-np.sin(theta_ref)) + (y - y_ref) * np.cos(theta_ref)

        return s, d

    def get_length(self):
        """Return total length of reference line"""
        return self.s_max

    def get_waypoints(self):
        """Return the scatter points defining this reference line"""
        return self.waypoints


def generate_straight_lane(x_start=0, y_start=0, length=100, heading=0):
    """
    Generate a straight lane reference line

    Args:
        x_start: starting x-coordinate
        y_start: starting y-coordinate
        length: length of the lane
        heading: heading angle in radians (0 = east, pi/2 = north)

    Returns:
        ReferenceLine object representing a straight lane
    """
    # Generate waypoints along straight line
    num_points = 100
    s = np.linspace(0, length, num_points)
    x = x_start + s * np.cos(heading)
    y = y_start + s * np.sin(heading)

    waypoints = np.column_stack([x, y])
    return ReferenceLine(waypoints)


def generate_sine_lane(x_start=0, y_start=0, length=100, amplitude=5, frequency=0.1):
    """
    Generate a sine-wave lane reference line

    Args:
        x_start: starting x-coordinate
        y_start: starting y-coordinate
        length: length of the lane
        amplitude: amplitude of sine wave
        frequency: frequency of sine wave

    Returns:
        ReferenceLine object representing a sine-wave lane
    """
    # Generate waypoints along sine wave
    num_points = 100
    s = np.linspace(0, length, num_points)
    x = x_start + s
    y = y_start + amplitude * np.sin(frequency * s)

    waypoints = np.column_stack([x, y])
    return ReferenceLine(waypoints)


def generate_s_turn_lane(x_start=0, y_start=0, length=100, amplitude=10):
    """
    Generate an S-turn lane reference line

    Args:
        x_start: starting x-coordinate
        y_start: starting y-coordinate
        length: length of the lane
        amplitude: amplitude of S-turn

    Returns:
        ReferenceLine object representing an S-turn lane
    """
    # Generate waypoints along S-turn
    num_points = 100
    s = np.linspace(0, length, num_points)
    x = x_start + s
    y = y_start + amplitude * np.sin(2 * np.pi * s / length)

    waypoints = np.column_stack([x, y])
    return ReferenceLine(waypoints)


class Lane:
    """
    Represents a single lane with its reference line and metadata
    """

    def __init__(self, lane_id, reference_line, lane_type='normal', left_neighbor=None, right_neighbor=None):
        """
        Initialize a lane

        Args:
            lane_id: unique identifier for this lane (int)
            reference_line: ReferenceLine object representing the lane center
            lane_type: type of lane ('normal', 'shoulder_left', 'shoulder_right')
            left_neighbor: Lane object for the lane to the left
            right_neighbor: Lane object for the lane to the right
        """
        self.id = lane_id
        self.reference_line = reference_line
        self.lane_type = lane_type
        self.left_neighbor = left_neighbor
        self.right_neighbor = right_neighbor

    def get_left_lane(self):
        """Get the lane to the left"""
        return self.left_neighbor

    def get_right_lane(self):
        """Get the lane to the right"""
        return self.right_neighbor

    def has_left_lane(self):
        """Check if there is a lane to the left"""
        return self.left_neighbor is not None

    def has_right_lane(self):
        """Check if there is a lane to the right"""
        return self.right_neighbor is not None


class ReferenceLineSystem:
    """
    Manages multiple reference lines (lanes) with topological relationships

    Represents the road network as a collection of lanes with logical connections.
    Supports lane navigation and queries.
    """

    def __init__(self, road_type='straight', road_length=100, lane_width=4.0, num_lanes=3):
        """
        Initialize reference line system

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

        # Generate lanes
        self.lanes = []
        self._generate_lanes()

        # Build lane topology
        self._build_lane_topology()

    def _generate_lanes(self):
        """Generate reference lines for each lane"""
        if self.road_type == 'straight':
            base_lane = generate_straight_lane(length=self.road_length)
        elif self.road_type == 'sine':
            base_lane = generate_sine_lane(length=self.road_length, amplitude=5, frequency=0.1)
        elif self.road_type == 's_turn':
            base_lane = generate_s_turn_lane(length=self.road_length, amplitude=10)
        else:
            raise ValueError(f"Unknown road type: {self.road_type}")

        # Generate parallel lanes by offsetting perpendicular to the curve
        for i in range(self.num_lanes):
            # Calculate lateral offset for this lane
            # Center lane is at index num_lanes // 2
            center_idx = self.num_lanes // 2
            lateral_offset = (i - center_idx) * self.lane_width

            # Get base waypoints
            waypoints = base_lane.waypoints.copy()

            # Offset waypoints perpendicular to the curve
            offset_waypoints = []
            for j in range(len(waypoints)):
                x, y = waypoints[j]

                # Compute heading at this point
                if j < len(waypoints) - 1:
                    dx = waypoints[j+1, 0] - x
                    dy = waypoints[j+1, 1] - y
                else:
                    dx = x - waypoints[j-1, 0]
                    dy = y - waypoints[j-1, 1]

                heading = np.arctan2(dy, dx)

                # Normal direction (perpendicular to heading)
                normal_x = -np.sin(heading)
                normal_y = np.cos(heading)

                # Offset point
                offset_x = x + lateral_offset * normal_x
                offset_y = y + lateral_offset * normal_y

                offset_waypoints.append([offset_x, offset_y])

            offset_waypoints = np.array(offset_waypoints)
            ref_line = ReferenceLine(offset_waypoints)

            # Determine lane type
            if i == 0:
                lane_type = 'shoulder_right' if self.num_lanes > 1 else 'normal'
            elif i == self.num_lanes - 1:
                lane_type = 'shoulder_left' if self.num_lanes > 1 else 'normal'
            else:
                lane_type = 'normal'

            lane = Lane(lane_id=i, reference_line=ref_line, lane_type=lane_type)
            self.lanes.append(lane)

    def _build_lane_topology(self):
        """Build topological relationships between lanes"""
        for i, lane in enumerate(self.lanes):
            # Set left neighbor
            if i < self.num_lanes - 1:
                lane.left_neighbor = self.lanes[i + 1]

            # Set right neighbor
            if i > 0:
                lane.right_neighbor = self.lanes[i - 1]

    def get_lane(self, lane_id):
        """
        Get lane by ID

        Args:
            lane_id: lane identifier (int)

        Returns:
            Lane object
        """
        if 0 <= lane_id < len(self.lanes):
            return self.lanes[lane_id]
        else:
            raise IndexError(f"Lane ID {lane_id} out of range [0, {len(self.lanes)-1}]")

    def get_lane_by_position(self, x, y):
        """
        Find the closest lane to a given position

        Args:
            x: x-coordinate (float)
            y: y-coordinate (float)

        Returns:
            Lane object of the closest lane
        """
        min_dist = float('inf')
        closest_lane = None

        for lane in self.lanes:
            s, d = lane.reference_line.project_point(x, y)
            dist = abs(d)  # Lateral distance

            if dist < min_dist:
                min_dist = dist
                closest_lane = lane

        return closest_lane

    def get_all_lanes(self):
        """Return all lanes"""
        return self.lanes

    def get_num_lanes(self):
        """Return number of lanes"""
        return self.num_lanes

    def get_road_bounds(self):
        """
        Get road boundary coordinates

        Returns:
            tuple: (left_boundary_waypoints, right_boundary_waypoints)
        """
        if len(self.lanes) == 0:
            return None, None

        # Left boundary is the leftmost lane's left edge
        left_lane = self.lanes[-1]
        right_lane = self.lanes[0]

        # Get waypoints for boundaries
        left_waypoints = self._offset_lane_boundary(left_lane, offset=self.lane_width/2)
        right_waypoints = self._offset_lane_boundary(right_lane, offset=-self.lane_width/2)

        return left_waypoints, right_waypoints

    def _offset_lane_boundary(self, lane, offset):
        """
        Generate boundary waypoints by offsetting a lane

        Args:
            lane: Lane object
            offset: lateral offset (positive = left, negative = right)

        Returns:
            numpy array of boundary waypoints
        """
        waypoints = lane.reference_line.waypoints
        offset_waypoints = []

        for j in range(len(waypoints)):
            x, y = waypoints[j]

            # Compute heading at this point
            if j < len(waypoints) - 1:
                dx = waypoints[j+1, 0] - x
                dy = waypoints[j+1, 1] - y
            else:
                dx = x - waypoints[j-1, 0]
                dy = y - waypoints[j-1, 1]

            heading = np.arctan2(dy, dx)

            # Normal direction
            normal_x = -np.sin(heading)
            normal_y = np.cos(heading)

            # Offset point
            offset_x = x + offset * normal_x
            offset_y = y + offset * normal_y

            offset_waypoints.append([offset_x, offset_y])

        return np.array(offset_waypoints)