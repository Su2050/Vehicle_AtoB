import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# The 3D heuristic is taking a LONG time to compute!
# And it's not even helping that much (only 12/32 solved, down from 14/32).
# Why is it taking so long?
# Because I increased the step size to 0.5, but maybe it's expanding too many nodes?
# Actually, the step size was 0.25 in the original code.
# Let's revert everything in heuristic.py to the original state.
# I will use git checkout to restore it.

