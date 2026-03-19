import math
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from solvability_oracle import assess_problem_solvability, find_goal_staging_certificates


def test_solvability_oracle_clean_scene_is_likely_solvable():
    result = assess_problem_solvability(4.0, 0.8, math.radians(25.0), [])
    assert result["verdict"] == "LIKELY_SOLVABLE", result
    assert result["solvable"] is True, result
    assert result["terminal"]["certificate_count"] > 0, result
    assert result["terminal"]["best_certificate"] is not None, result


def test_solvability_oracle_far_obstacles_do_not_kill_terminal_certificate():
    obstacles = [
        {"x": 6.4, "y": 1.2, "w": 1.1, "h": 1.0},
        {"x": 6.9, "y": -1.7, "w": 0.8, "h": 1.2},
    ]
    terminal = find_goal_staging_certificates(obstacles)
    assert terminal["goal_pose_valid"] is True, terminal
    assert terminal["certificate_count"] > 0, terminal


def test_solvability_oracle_goal_blocked_is_likely_unsolvable_terminal():
    obstacles = [
        {"x": 1.85, "y": -0.35, "w": 0.90, "h": 0.90},
    ]
    result = assess_problem_solvability(4.5, 0.2, math.radians(-20.0), obstacles)
    assert result["verdict"] == "LIKELY_UNSOLVABLE_TERMINAL", result
    assert result["solvable"] is False, result
    assert result["terminal"]["goal_pose_valid"] is False, result


if __name__ == "__main__":
    test_solvability_oracle_clean_scene_is_likely_solvable()
    test_solvability_oracle_far_obstacles_do_not_kill_terminal_certificate()
    test_solvability_oracle_goal_blocked_is_likely_unsolvable_terminal()
    print("All solvability oracle tests passed!")
