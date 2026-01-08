# ADMM-iLQR Python Migration & Curved Road Support Plan

This document outlines the detailed plan to migrate the MATLAB-based ADMM-iLQR motion planner to **Python**, while simultaneously upgrading the architecture to support **curved roads** and **arbitrary lane geometries** using a Reference Line system.

## 🎯 Core Objectives

1.  **Language Migration**: MATLAB $\rightarrow$ Python (`numpy`, `matplotlib`, `scipy`).
2.  **Geometric Upgrade**: Support non-straight roads (curved lanes).
3.  **Lane Representation**: Transition from scalar Y-values to **Reference Lines** (polylines/splines).
4.  **Coordinate System**: Maintain the solver in the **Cartesian Frame $(x, y, \theta, v)$** (the solver logic remains largely unchanged, but the inputs change).

---

## 📅 Implementation Roadmap

### Phase 1: Infrastructure & Core Physics (Python Basis)
**Goal**: Establish the project skeleton and port the fundamental vehicle dynamics.

*   **1.1 Project Setup**
    *   Initialize `admm_ilqr_python/` directory.
    *   Dependencies: `numpy` (matrix math), `matplotlib` (plotting), `scipy` (interpolation), `joblib` (parallelism).
*   **1.2 Vehicle Dynamics (`vehicle_model.py`)**
    *   Port `update_state` (Kinematic Bicycle Model).
    *   Port `get_A_matrix` and `get_B_matrix` (Jacobians).
    *   *Critical Check*: Verify 0-based indexing (Python) vs 1-based (MATLAB).
*   **1.3 Cost Function (`cost.py`)**
    *   Port `get_cost_and_derivatives`.
    *   Logic remains $J = (x - x_{ref})^T Q (x - x_{ref})$.
    *   *Insight*: The solver naturally handles curves if $x_{ref}$ traces a curve.

### Phase 2: Reference Line System (The "Curved Road" Enabler)
**Goal**: Replace scalar lane definitions with geometric curves.

*   **2.1 ReferenceLine Class (`reference_line.py`)**
    *   **Input**: A list of global waypoints `[(x0, y0), (x1, y1), ...]`. 
    *   **Representation**: Use `scipy.interpolate.CubicSpline` or cumulative distance interpolation.
    *   **Key Method**: `get_state_at_s(s) -> (x, y, theta, kappa)`
        *   Given longitudinal distance $s$, return the global state on the curve.
    *   **Key Method**: `project_point(x, y) -> (s, d)`
        *   Find the frenet coordinates of the vehicle relative to this line.
*   **2.2 Scenario Generation (`scenario.py`)**
    *   Create a generator for arbitrary road geometries (e.g., Sine Wave, S-Turn).
    *   Define lanes as a list of `ReferenceLine` objects (e.g., `[left_lane, center_lane, right_lane]`).

### Phase 3: Trajectory Candidate Generation
**Goal**: Generate guesses that follow the curved roads.

*   **3.1 Longitudinal Planning**
    *   Plan velocity/position profile in $s$-domain: $s(t)$.
    *   This remains 1D and simple (Cruise, Stop, Follow).
*   **3.2 Lateral Mapping (Frenet $\rightarrow$ Cartesian)**
    *   **Lane Keep**:
        *   Combine $s(t)$ with lateral offset $d(t) \approx 0$.
        *   Map $(s(t), 0)$ back to global $(x_{ref}, y_{ref})$ using `ReferenceLine`.
    *   **Lane Change**:
        *   Define `start_lane` and `target_lane`.
        *   Generate a smooth transition path in Cartesian space connecting the two reference lines, OR define a virtual polynomial connection in Frenet space.
    *   *Result*: A list of `Candidate` objects, each containing a curved `x_ref` trajectory.

### Phase 4: ADMM-iLQR Optimizer Migration
**Goal**: Port the heavy-lifting optimization engine.

*   **4.1 Constraint Projection (`constraints.py`)**
    *   Port `project_constraints`.
    *   **Obstacles**: The ellipsoidal projection logic works in $(x, y)$ global space, so it **needs NO modification** for curved roads. It just works.
    *   **Road Boundaries**: Update to check `abs(d) < lane_width/2` using `ReferenceLine.project_point` instead of simple `y` limits.
*   **4.2 iLQR Solver (`ilqr.py`)**
    *   Port `forward_pass` (line search) and `backward_pass` (Riccati recursion).
    *   Ensure matrix multiplications use `@` operator.
*   **4.3 ADMM Loop (`optimizer.py`)**
    *   Implement the outer ADMM iteration loop.
    *   Use `joblib.Parallel` to optimize multiple candidates simultaneously (replacing `parfor`).

### Phase 5: Simulation & Visualization
**Goal**: Verify the system in a closed loop.

*   **5.1 Simulation Loop (`main.py`)**
    *   Initialize `Scenario` (Curved).
    *   Loop: `Planner` $\rightarrow$ `Step Physics` $\rightarrow$ `Update Visualization`.
*   **5.2 Visualization**
    *   Plot the curved `ReferenceLines` (gray dashed lines).
    *   Plot the predicted trajectory (colored line).
    *   Plot obstacles and the ego vehicle.

---

## 🔑 Key Technical Differences

| Feature | MATLAB (Original) | Python (New) |
| :--- | :--- | :--- |
| **Lane Definition** | Scalar `y = [-4, 0, 4]` | `ReferenceLine` objects (Splines/Polylines) |
| **Lane Keep Logic** | `y_ref = constant` | `x_ref, y_ref = RefLine.get_state_at_s(s)` |
| **Lane Change Logic** | Interpolate between two Y-values | Interpolate between two geometric curves |
| **Matrix Lib** | Native MATLAB | `numpy` |
| **Parallelism** | `parfor` | `joblib` or `multiprocessing` |
| **Structure** | Structs + Functions | Classes (`Vehicle`, `Planner`, `RefLine`) |

## 📝 Checkpoints for Success

1.  **Reference Line Math**: Verify that `get_state_at_s` accurately returns points and headings along a curve.
2.  **Projection**: Verify that obstacles are correctly avoided even when the road is turning (since the planner works in Global X-Y, this should be robust).
3.  **Tracking**: Verify that with zero obstacles, the vehicle can perfectly track a sine-wave lane center using the `KeepLane` candidate.
