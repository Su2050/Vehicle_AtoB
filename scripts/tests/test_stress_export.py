import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_stress import _classify_failure_bucket, _timing_record_from_result


def test_timing_record_exports_strategy_fields():
    """timing record 应保留 late-merge / deep-stage 诊断字段。"""
    result = {
        'id': 7,
        'type': 'SlalomStaggered',
        'ok': False,
        'timed_out': False,
        'expect_fail': False,
        'collision': False,
        'elapsed_ms': 1234.56,
        'expanded': 4321,
        'level': 'FAILED_all_stages',
        'replay_primitive_profile': 'slalom',
        'oracle_verdict': 'LIKELY_SOLVABLE',
        'oracle_solvable': True,
        'oracle_rationale': 'Found a goal-front staging certificate and a finite 2D route to it.',
        'oracle_candidate_count': 400,
        'oracle_aligned_free_pose_count': 126,
        'oracle_local_box_clear_count': 93,
        'oracle_certificate_count': 12,
        'oracle_best_stage_pose': (2.65, 0.05, 4.0),
        'oracle_best_stage_insert_len': 0.553,
        'oracle_best_stage_reachable_2d': True,
        'oracle_best_stage_2d_dist': 1.844,
        'failure_bucket': 'LIKELY_MISSED_SOLVABLE',
        'failure_bucket_reason': 'staging_certificate_and_2d_reachability',
        'pure_rs_first_invalid_reason': 'CORRIDOR',
        'pure_rs_first_invalid_idx': 62,
        'pure_rs_first_invalid_pt': (2.048, 0.167, 0.224),
        'l15_locality_mode': 'near_goal_local',
        'l15_relevant_obstacle_count': 0,
        'l15_skipped_reason': 'no_local_obstacles',
        'late_merge_gate_attempted': True,
        'late_merge_gate_stage_pose': (4.66, 0.65, 1.32),
        'late_merge_deep_attempted': True,
        'late_merge_deep_start_kind': 'gate_stage',
        'late_merge_deep_expanded': 9500,
    }

    record = _timing_record_from_result(result)

    assert record['category'] == 'UNEXPECTED_FAIL'
    assert record['case_id'] == 7
    assert record['type'] == 'SlalomStaggered'
    assert record['replay_primitive_profile'] == 'slalom'
    assert record['oracle_verdict'] == 'LIKELY_SOLVABLE'
    assert record['oracle_certificate_count'] == 12
    assert record['oracle_best_stage_pose'] == (2.65, 0.05, 4.0)
    assert record['failure_bucket'] == 'LIKELY_MISSED_SOLVABLE'
    assert record['pure_rs_first_invalid_reason'] == 'CORRIDOR'
    assert record['pure_rs_first_invalid_idx'] == 62
    assert record['l15_locality_mode'] == 'near_goal_local'
    assert record['l15_relevant_obstacle_count'] == 0
    assert record['l15_skipped_reason'] == 'no_local_obstacles'
    assert record['late_merge_gate_attempted'] is True
    assert record['late_merge_gate_stage_pose'] == (4.66, 0.65, 1.32)
    assert record['late_merge_deep_attempted'] is True
    assert record['late_merge_deep_start_kind'] == 'gate_stage'
    assert record['late_merge_deep_expanded'] == 9500


def test_failure_bucket_mapping():
    assert _classify_failure_bucket('LIKELY_UNSOLVABLE_TERMINAL')[0] == 'TERMINAL_BLOCKED'
    assert _classify_failure_bucket('LIKELY_SOLVABLE')[0] == 'LIKELY_MISSED_SOLVABLE'
    assert _classify_failure_bucket('TERMINAL_INSERT_CERTIFIED')[0] == 'TRUE_HARD_CASE'
    assert _classify_failure_bucket('UNKNOWN')[0] == 'TRUE_HARD_CASE'


if __name__ == "__main__":
    test_timing_record_exports_strategy_fields()
    test_failure_bucket_mapping()
    print("All stress export tests passed!")
