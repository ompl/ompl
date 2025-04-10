import sys
import pytest

from ompl import base as ob
from ompl import geometric as og

def test_rrt_planner():
    # 1) Create a 2D RealVectorStateSpace
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # 2) Create a trivial validity checker that always returns True.
    # For demonstration, we won't do real collision checks.
    def is_valid(state):
        return True
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    # 3) Create a ProblemDefinition with a start and goal state.
    start = ob.RealVector.ScopedState(space)
    start[0] = -0.5
    start[1] = -0.5
    goal = ob.RealVector.ScopedState(space)
    goal[0] = 0.5
    goal[1] = 0.5

    # 4) Create the RRT planner.
    # Let's choose addIntermediateStates=True for demonstration.
    rrt_planner = og.RRT(si, True)
    # 5) Configure some parameters
    rrt_planner.setGoalBias(0.1)
    rrt_planner.setRange(0.2)

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(rrt_planner)

    # Print them out
    print("Goal bias:", rrt_planner.getGoalBias())
    print("Range:", rrt_planner.getRange())
    print("Intermediate states:", rrt_planner.getIntermediateStates())


    # 7) Construct a PlannerTerminationCondition that stops after 1 second.
    ptc = ob.PlannerTerminationCondition(lambda: False, 1.0)

    # 8) Solve
    result = ss.solve(ptc)
    print("Planner result:", result)

    # 9) The `result` is a PlannerStatus object. Check if solution found:
    if result:
        print("Solution found!")
        # Optionally, retrieve the PathGeometric:
        solutionPath = ss.getSolutionPath()
        if solutionPath:
            print("Solution path length:", solutionPath.length())
            print("Solution path states:", solutionPath.getStateCount())
    else:
        print("No solution found within 1 second of planning time.")
