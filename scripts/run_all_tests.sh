#!/bin/bash
# 不使用 set -e，以便单个测试失败不阻断后续测试
cd "$(dirname "$0")"

echo "============================================================"
echo " [1/4] Stress Test (thorough, 8 workers)"
echo "============================================================"
python3 tests/test_stress.py --profile thorough --workers 8
echo ""

echo "============================================================"
echo " [2/4] Obstacle Planning Scenarios (test_obs_v2)"
echo "============================================================"
python3 tests/test_obs_v2.py
echo ""

echo "============================================================"
echo " [3/4] Path Quality Acceptance (test_path_quality)"
echo "============================================================"
python3 tests/test_path_quality.py
echo ""

echo "============================================================"
echo " [4/4] Path Geometry Quality (test_path_quality_v2)"
echo "============================================================"
python3 tests/test_path_quality_v2.py
echo ""

echo "============================================================"
echo " ALL 4 TEST SUITES COMPLETED"
echo "============================================================"
