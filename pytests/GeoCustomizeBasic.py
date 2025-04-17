from ompl import base as ob 
from ompl import geometric as og
import math
import pytest
import time
import random
# Note: virtual bool isSatisfied (const State *st, double *distance) const is not available in the python API. 
#   Refer to GoalRegion if you want to use this function.
class MyGoal(ob.Goal):
    def __init__(self, si: ob.SpaceInformation):
        super().__init__(si)
        self.si = si

    def isSatisfied(self, state: ob.State) -> bool:
        # the second dimension of the state is [-0.1, 0.1]
        val = state[1]
        if -0.1 < val < 0.1:
            return True
        return False
    
    def isStartGoalPairValid(self, start: ob.State, goal: ob.State) -> bool:
        return True
    
    def print(self):
        print("MyGoal")

def test_customize_goal():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    def is_valid(state):
        return True
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    goal = MyGoal(si)
    validState = si.allocState()
    validState[0] = 0.0
    validState[1] = 0.0
    val = goal.isSatisfied(validState)
    print("Is goal satisfied? ", val)
    assert val, "Goal should be satisfied"

    invalidState = si.allocState()
    invalidState[0] = 0.0
    invalidState[1] = 0.2
    val = goal.isSatisfied(invalidState)
    print("Is goal satisfied? ", val)
    assert not val, "Goal should not be satisfied"

class MyGoalRegion(ob.GoalRegion):
    def __init__(self, si: ob.SpaceInformation):
        super().__init__(si)
        self.si = si

    def distanceGoal(self, state: ob.State) -> float:
        # the second dimension of the state is [-0.1, 0.1]
        val = state[1]
        if -0.1 < val < 0.1:
            return 0.0
        return abs(val) - 0.1

def test_customize_goal_region():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    def is_valid(state):
        return True
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    goal = MyGoalRegion(si)
    goal.setThreshold(1E-4)
    validState = si.allocState()
    validState[0] = 0.0
    validState[1] = 0.0
    val = goal.isSatisfied(validState)
    si.printState(validState)
    print("Distance to goal: ", goal.distanceGoal(validState))
    print("Is goal satisfied? ", val)
    assert val, "Goal should be satisfied"

    invalidState = si.allocState()
    invalidState[0] = 0.0
    invalidState[1] = 0.2
    si.printState(invalidState)
    distance = goal.distanceGoal(invalidState)
    assert distance == pytest.approx(0.1), "Distance should be 0.1"
    print("Distance to goal: ", goal.distanceGoal(invalidState))
    print("Is goal satisfied? ", val)
    val = goal.isSatisfied(invalidState)
    assert not val, "Goal should not be satisfied"

class GoalSampleableRegion(ob.GoalSampleableRegion):
    def __init__(self, si: ob.SpaceInformation):
        super().__init__(si)
        self.si = si
    def sampleGoal(self, state: ob.State) -> None:
        state[0] = 0.0
        state[1] = random.uniform(-0.1, 0.1)
    def maxSampleCount(self) -> int:
        return 10
    def couldSample(self) -> bool:
        return True
    def distanceGoal(self, state: ob.State) -> float:
        # the second dimension of the state is [-0.1, 0.1]
        val = state[1]
        if -0.1 < val < 0.1:
            return 0.0
        return abs(val) - 0.1
    
def test_customize_goal_sampleable_region():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    def is_valid(state):
        return True
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    goal = GoalSampleableRegion(si)
    goal.setThreshold(1E-4)
    print(goal.getThreshold())
    validState = si.allocState()
    validState[0] = 0.0
    validState[1] = 0.0
    val = goal.isSatisfied(validState)
    dis = goal.isSatisfiedWithDistance(validState)
    print("Is goal satisfied? ", val)
    print("Distance to goal: ", dis)
    assert val, "Goal should be satisfied"

    invalidState = si.allocState()
    invalidState[0] = 0.0
    invalidState[1] = 0.2
    val = goal.isSatisfied(invalidState)
    print("Is goal satisfied? ", val)
    assert not val, "Goal should not be satisfied"

    rrt_planner = og.RRT(si, True)
    rrt_planner.setGoalBias(0.1)
    rrt_planner.setRange(0.2)
    ss = og.SimpleSetup(si)
    ss.setGoal(goal)
    ss.setStartState(validState)
    ss.setPlanner(rrt_planner)

    start_time = time.time()
    # Create a termination condition that returns True after 5 seconds
    ptc = ob.PlannerTerminationCondition(
        lambda: (time.time() - start_time) > 5
    )
    # Note: 
    result = ss.solve(ptc)
    print("Planner result:", result)

if __name__ == "__main__":
    test_customize_goal()
    test_customize_goal_region()
    test_customize_goal_sampleable_region()