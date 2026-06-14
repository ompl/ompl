import pytest

from ompl import base as ob
from ompl import multilevel as ml

from geo_env import create_simple_setup, solve_with_planner


def test_multilevel_imports():
    assert ml.QRRT is not None
    assert ml.QRRTStar is not None
    assert ml.QMP is not None
    assert ml.QMPStar is not None
    assert ml.ProjectionType is not None
    assert ml.FindSectionType is not None


def test_qrrt_single_level():
    """QRRT with one SpaceInformation falls back to standard planning."""
    ss = create_simple_setup()
    si = ss.getSpaceInformation()
    planner = ml.QRRT(si)
    solution_path = solve_with_planner(ss, planner, timeout=2.0)
    assert solution_path is not None


def test_qrrt_multilevel():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    def is_valid(_state):
        return True

    si_vec = []
    for _ in range(2):
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(is_valid)
        si.setup()
        si_vec.append(si)

    start = si_vec[-1].allocState()
    start[0] = -0.5
    start[1] = -0.5
    goal = si_vec[-1].allocState()
    goal[0] = 0.5
    goal[1] = 0.5

    pdef = ob.ProblemDefinition(si_vec[-1])
    pdef.setStartAndGoalStates(start, goal)

    planner = ml.QRRT(si_vec)
    planner.setProblemDefinition(pdef)
    planner.setup()
    result = planner.solve(2.0)
    assert result
    assert pdef.hasSolution()
    assert pdef.getSolutionPath() is not None
