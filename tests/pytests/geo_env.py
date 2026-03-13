import time

from ompl import base as ob
from ompl import geometric as og


def create_simple_setup():
    """Create and return a SimpleSetup for a 2D planning problem."""
    # 1) Create a 2D RealVectorStateSpace
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # 2) Create a trivial validity checker that always returns True.
    def is_valid(state):
        return True

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    # 3) Create start and goal states
    start = si.allocState()
    start[0] = -0.5
    start[1] = -0.5
    goal = si.allocState()
    goal[0] = 0.5
    goal[1] = 0.5

    # 4) Create SimpleSetup with start and goal
    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)

    return ss


def solve_with_planner(ss, planner, timeout=5.0):
    """
    Set the planner on SimpleSetup, solve, and check the result.
    
    Args:
        ss: SimpleSetup object
        planner: The planner to use
        timeout: Maximum solve time in seconds (default 5.0)
    
    Returns:
        The solution path if found, None otherwise
    """
    ss.setPlanner(planner)
    
    # Solve
    result = ss.solve(timeout)
    print("Planner result:", result)

    # Check if solution found
    if result:
        print("Solution found!")
        solution_path = ss.getSolutionPath()
        if solution_path:
            print("Solution path length:", solution_path.length())
            print("Solution path states:", solution_path.getStateCount())
        return solution_path
    else:
        print(f"No solution found within {timeout} seconds of planning time.")
        return None

