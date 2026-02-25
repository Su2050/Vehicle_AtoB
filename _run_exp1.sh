#!/bin/bash
cd "/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts"
python3 tests/test_obs_v2.py --planner v1 --scenario S12_goal_covered --timeout 30 > "/Users/sull/Documents/untitled folder/Vehicle_AtoB/_exp1_out.txt" 2>&1
echo "EXIT_CODE=$?" >> "/Users/sull/Documents/untitled folder/Vehicle_AtoB/_exp1_out.txt"
