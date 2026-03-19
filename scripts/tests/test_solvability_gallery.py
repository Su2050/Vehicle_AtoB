import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sample_solvability_oracle_gallery import _balanced_sample_ids


def test_balanced_sample_ids_keeps_all_verdicts():
    buckets = {
        "A": [1, 2, 3, 4],
        "B": [5, 6, 7],
        "C": [8, 9],
    }
    selected = _balanced_sample_ids({k: list(v) for k, v in buckets.items()}, 6, __import__("random").Random(7))
    assert len(selected) == 6
    assert any(v in selected for v in buckets["A"])
    assert any(v in selected for v in buckets["B"])
    assert any(v in selected for v in buckets["C"])


if __name__ == "__main__":
    test_balanced_sample_ids_keeps_all_verdicts()
    print("All solvability gallery tests passed!")
