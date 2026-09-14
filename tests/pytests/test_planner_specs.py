from ompl import base as ob
from ompl import geometric as og

from geo_env import create_simple_setup


def test_planner_specs_access():
    ss = create_simple_setup()
    planner = og.RRT(ss.getSpaceInformation())
    specs = planner.getSpecs()
    assert specs is not None
    assert specs.multithreaded is False
    assert specs.optimizingPaths is False
