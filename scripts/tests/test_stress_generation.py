import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_stress import generate_cases


def test_generate_cases_includes_slalom_batch():
    """quick 档应显式生成 slalom 专项批量集。"""
    cases = generate_cases(profile="quick", seed=42)
    slalom = [c for c in cases if c["type"] == "SlalomStaggered"]

    assert slalom, "Expected SlalomStaggered cases in quick profile"
    assert len(slalom) >= 20, len(slalom)
    assert all(len(c["obstacles"]) == 3 for c in slalom[:10]), slalom[:3]
    assert any(c["y"] > 1.5 for c in slalom), "Expected upper-side slalom starts"
    assert any(c["y"] < -1.5 for c in slalom), "Expected lower-side slalom starts"


if __name__ == "__main__":
    test_generate_cases_includes_slalom_batch()
    print("All stress generation tests passed!")
