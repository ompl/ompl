import sys
import pytest
import math
# Make sure the OMPL bindings are in the Python path.
from ompl import base as ob

def test_goal():
    print(ob.GOAL_ANY.value)
    assert ob.GOAL_ANY == ob.GoalType.GOAL_ANY

if __name__ == "__main__":
    test_goal()