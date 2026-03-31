import pytest

from ompl import geometric as og

from geo_env import create_simple_setup, solve_with_planner


def test_rrt():
    # 1) Create SimpleSetup from environment
    ss = create_simple_setup()
    si = ss.getSpaceInformation()

    # 2) Create the RRT planner
    rrt_planner = og.RRT(si, True)
    rrt_planner.setGoalBias(0.1)
    rrt_planner.setRange(0.2)

    # Print planner settings
    print("Goal bias:", rrt_planner.getGoalBias())
    print("Range:", rrt_planner.getRange())
    print("Intermediate states:", rrt_planner.getIntermediateStates())

    # 3) Solve and check result
    solution_path = solve_with_planner(ss, rrt_planner)
    assert solution_path is not None

def test_rrt_connect():
    # 1) Create SimpleSetup from environment
    ss = create_simple_setup()
    si = ss.getSpaceInformation()

    # 2) Create the RRTConnect planner
    rrt_connect_planner = og.RRTConnect(si, True)
    rrt_connect_planner.setRange(0.2)

    # 3) Solve and check result
    solution_path = solve_with_planner(ss, rrt_connect_planner)
    assert solution_path is not None

def test_prm():
    # 1) Create SimpleSetup from environment
    ss = create_simple_setup()
    si = ss.getSpaceInformation()

    # 2) Create the PRM planner
    prm_planner = og.PRM(si, False)

    # 3) Solve and check result
    solution_path = solve_with_planner(ss, prm_planner)
    assert solution_path is not None
    print("Expanding roadmap")
    ss.getPlanner().expandRoadmap(0.01)
    print("Growing roadmap")
    ss.getPlanner().growRoadmap(0.01)
    print("Constructing roadmap")
    ss.getPlanner().constructRoadmap(0.01)
    print("Clearing roadmap")
    ss.getPlanner().clear()


def test_prm_star():
    # 1) Create SimpleSetup from environment
    ss = create_simple_setup()
    si = ss.getSpaceInformation()

    # 2) Create the PRMstar planner
    prm_star_planner = og.PRMstar(si)

    # 3) Solve and check result
    solution_path = solve_with_planner(ss, prm_star_planner, timeout=0.01)
    assert solution_path is not None

if __name__ == "__main__":
    test_rrt()
    test_rrt_connect()
    test_prm()
    test_prm_star()
