# ADMM-iLQR Autonomous Driving Motion Planner (MATLAB)

## Project Overview

This project implements a **motion planning library for autonomous driving** using a **Constrained Iterative Linear Quadratic Regulator (iLQR)** based on the **Alternating Direction Method of Multipliers (ADMM)**.

The core objective is to efficiently plan safe and smooth trajectories for an autonomous vehicle in dynamic environments, handling constraints like:
- Nonlinear vehicle dynamics (Kinematic Bicycle Model).
- Obstacle avoidance (Static and Dynamic).
- Road boundaries.
- Control limits (steering and acceleration).

The system supports **Multimodal Parallel Planning**, generating and optimizing multiple candidate trajectories (e.g., Lane Keeping, Lane Change, Car Following) simultaneously to handle complex traffic scenarios.

## Key Features

*   **ADMM-iLQR Optimizer:** Efficiently handles hard constraints within the iLQR framework.
*   **Parallel Planning:** Simultaneously optimizes multiple behavioral candidates (Cruising, Lane Change, Stopping) to find the best maneuver.
*   **Closed-Loop Simulation:** Includes a built-in traffic simulator to test algorithm stability over long horizons.
*   **Constraint Projections:** Implements various projection methods for obstacles (Euclidean, Ray-Casting) and stability barriers.

## Building and Running

This is a pure MATLAB project. No compilation is required.

### Prerequisites

*   **MATLAB R2020b** or later.
*   **Parallel Computing Toolbox** (Recommended): Required for parallel trajectory optimization in `main_ADMM_iLQR.m`.
    *   *Note:* If the toolbox is not available, `parfor` loops must be manually changed to `for` loops.

### Running the Planner

1.  **Single Frame Demo:**
    To visualize the planning process for a single time step:
    ```matlab
    >> main_ADMM_iLQR
    ```
    *   Generates a 3-lane scenario.
    *   Optimizes multiple candidate trajectories in parallel.
    *   Selects and plots the best trajectory.

2.  **Closed-Loop Simulation:**
    To run a continuous simulation with traffic flow:
    ```matlab
    >> run_closed_loop_simulation
    ```
    *   Simulates vehicle dynamics and traffic flow over time.
    *   Continuously plans and controls the ego vehicle.
    *   Displays an animation of the results and saves metrics to `sim_results.mat`.

## File Structure

### Entry Points
*   `main_ADMM_iLQR.m`: **Start here.** Script for running a single-frame planning demonstration.
*   `run_closed_loop_simulation.m`: Script for running a continuous closed-loop simulation.

### Core Algorithm
*   `run_admm_ilqr.m`: The main ADMM loop (Algorithm 1) that coordinates the optimization.
*   `run_iLQR_admm_wrapper.m`: The inner iLQR solver.
*   `forward_pass_admm.m` / `backward_pass_admm.m`: Core steps of the iLQR algorithm.

### Constraints & Physics
*   `project_constraints.m`: Main interface for constraint projection.
*   `project_constraints_*.m`: Specific implementations for different constraint types (e.g., `_euclidean`, `_ray_casting`).
*   `get_cost_and_derivatives.m`: Computes cost functions and their derivatives for the optimization.

### Simulation & Helpers
*   `generate_candidates_structured.m`: Generates high-level behavioral candidates (e.g., "Lane Change Left", "Follow").
*   `evaluate_trajectories.m`: Scores optimized trajectories to select the best one based on safety, progress, and comfort.
*   `build_simulation_scenario.m`: Sets up the road and initial obstacles.
*   `update_obstacle_physics.m` / `update_traffic_manager.m`: Manages traffic flow in the simulation.

## Development Conventions

*   **Language:** MATLAB (.m).
*   **Parallelism:** `parfor` is used for high-level parallelization of candidate trajectories.
*   **Configuration:** Simulation and planner parameters (weights, horizons, etc.) are defined at the top of the entry scripts (`main_ADMM_iLQR.m` and `run_closed_loop_simulation.m`).
*   **Visualization:** Plotting functions are separated into files like `plot_results.m` and `plot_iteration.m`.
