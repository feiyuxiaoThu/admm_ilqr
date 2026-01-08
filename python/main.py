"""
Main Simulation Loop - ADMM-iLQR Motion Planner
Ported from MATLAB main_ADMM_iLQR.m
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from geometry.scenario import build_simulation_scenario
from planning.candidate_generator import generate_candidates_structured
from planning.optimizer import run_admm_ilqr
from utils.evaluator import evaluate_trajectories
from utils.visualization import plot_results_multimodal


def main():
    """
    Main simulation loop for ADMM-iLQR motion planner
    """
    # Global parameters
    dt = 0.1              # time step (s)
    L = 3.0               # wheelbase (m)
    v_target = 10.0       # target speed (m/s)
    total_time = 6.0      # total simulation time (s)
    N = int(total_time / dt)

    # iLQR optimization weights
    weights = {
        'q_pos_x': 0.1,           # position x penalty (stage)
        'q_pos_y': 0.2,           # position y penalty (stage)
        'q_vel': 0.0,             # velocity penalty (stage)
        'r_acc': 0.5,             # acceleration penalty
        'r_steer': 50.0,          # steering penalty
        'q_pos_x_term': 0.1,      # terminal x penalty
        'q_pos_y_term': 50.0,     # terminal y penalty
        'q_vel_term': 0.0,        # terminal velocity penalty
        'r_delta_acc': 10.0,      # control rate penalty (acceleration)
        'r_delta_steer': 10.0     # control rate penalty (steering)
    }

    # Trajectory evaluation weights (for selecting best candidate)
    eval_weights = {
        'w_safety': 2.0,          # safety margin penalty
        'w_progress': 10.0,       # progress reward
        'w_ref_vel': 0.5,         # velocity error penalty
        'w_acc': 0.1,             # acceleration penalty (comfort)
        'w_steer': 100.0,         # steering penalty
        'w_jerk_lon': 0.1,        # longitudinal jerk penalty
        'w_jerk_lat': 100.0,      # lateral jerk penalty
        'w_consistency': 500.0    # decision consistency (avoid frequent switching)
    }
    last_best_id = -1
    last_best_cand_name = ''

    # ADMM-iLQR algorithm parameters
    options = {
        'max_admm_iter': 10,      # max ADMM iterations
        'sigma': 10.0,            # ADMM penalty parameter (initial)
        'tol_admm': 1e-1,         # ADMM convergence tolerance
        'max_ilqr_iter': 50,      # max iLQR iterations per ADMM step
        'ilqr_tol': 1e-1          # iLQR convergence tolerance
    }

    # Build simulation scenario
    print("Building simulation scenario...")
    scenario, constraints, x0, raw_obs_list = build_simulation_scenario(dt, N, road_type='straight')

    # Generate candidate trajectories
    print(f"Generating candidate trajectories...")
    candidates = generate_candidates_structured(x0, scenario, constraints, N, dt)
    print(f"Generated {len(candidates)} candidate trajectories.")

    # Optimize all candidates
    results = []
    print(f"\nOptimizing candidates...")
    for i, candidate in enumerate(candidates):
        v_target = candidate.v_target.item() if hasattr(candidate.v_target, 'item') else float(candidate.v_target)
        print(f"Optimizing Candidate {candidate.id}: {candidate.name} (Target V={v_target:.2f})")

        X_opt, U_opt, debug_info = run_admm_ilqr(
            x0, candidate, constraints, dt, L, weights, options
        )

        result = {
            'X': X_opt,
            'U': U_opt,
            'cost': debug_info.get('cost_history', [0])[-1] if debug_info.get('cost_history') else 0,
            'cand': candidate,
            'debug_info': debug_info
        }
        results.append(result)

    print(f"\nAll candidates optimized.")

    # Evaluate trajectories and select best candidate
    scenario_params = {'v_desired': scenario.v_desired}
    print("\nEvaluating trajectories...")
    best_idx, best_score, all_scores = evaluate_trajectories(
        results,
        constraints['obstacles'],  # list of obstacles
        last_best_id,
        last_best_cand_name,
        scenario_params,
        eval_weights
    )

    # Output decision result
    if best_idx != -1:
        best_cand = results[best_idx]['cand']
        score_info = all_scores[best_idx]
        print(f"\n>>> FINAL DECISION: Candidate {best_cand.id} ({best_cand.name})")
        print(f"    Score: {best_score:.4f}")
        print(f"      Safety: {score_info['J_safe'] * eval_weights['w_safety']:.1f}")
        print(f"      Progress: {score_info['J_prog']:.1f}")
        print(f"      Comfort: {score_info['J_comf']:.1f}")

        # Extract optimal trajectory for execution
        final_X = results[best_idx]['X']
        final_U = results[best_idx]['U']
    else:
        print("\nWARNING: No valid trajectory found! Triggering AEB.")

    # Visualization
    if best_idx != -1:
        print("\nGenerating visualization...")
        plot_results_multimodal(results, best_idx, constraints, scenario, dt)
    else:
        print("No valid trajectory to plot.")

    return results, best_idx, all_scores


if __name__ == '__main__':
    results, best_idx, scores = main()
    print("\nSimulation complete!")
