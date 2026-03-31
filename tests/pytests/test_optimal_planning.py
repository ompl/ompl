import pytest
import math
from ompl import base as ob
from ompl import geometric as og


def create_simple_setup():
    """Create a simple 2D planning problem."""
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)

    def is_valid(state):
        # Simple validity checker - avoid center obstacle
        x, y = state[0], state[1]
        # Obstacle from (0.3, 0.3) to (0.7, 0.7)
        if 0.3 < x < 0.7 and 0.3 < y < 0.7:
            return False
        return True

    si.setStateValidityChecker(is_valid)
    si.setup()

    ss = og.SimpleSetup(si)

    start = si.allocState()
    start[0] = 0.1
    start[1] = 0.1

    goal = si.allocState()
    goal[0] = 0.9
    goal[1] = 0.9

    ss.setStartAndGoalStates(start, goal)

    # Set optimization objective (path length)
    objective = ob.PathLengthOptimizationObjective(si)
    ss.setOptimizationObjective(objective)

    return ss, si


def test_rrtstar():
    ss, si = create_simple_setup()

    planner = og.RRTstar(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    solved = ss.solve(1.0)
    assert solved, "RRTstar should find a solution"


def test_informed_rrtstar():
    ss, si = create_simple_setup()

    planner = og.InformedRRTstar(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    solved = ss.solve(1.0)
    assert solved, "InformedRRTstar should find a solution"


def test_sorrtstar():
    ss, si = create_simple_setup()

    planner = og.SORRTstar(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    solved = ss.solve(1.0)
    assert solved, "SORRTstar should find a solution"


def test_bitstar():
    ss, si = create_simple_setup()

    planner = og.BITstar(si)
    planner.setSamplesPerBatch(100)
    ss.setPlanner(planner)

    solved = ss.solve(1.0)
    assert solved, "BITstar should find a solution"


def test_fmt():
    ss, si = create_simple_setup()

    planner = og.FMT(si)
    planner.setNumSamples(500)
    ss.setPlanner(planner)

    solved = ss.solve(2.0)
    assert solved, "FMT should find a solution"


def test_bfmt():
    ss, si = create_simple_setup()

    planner = og.BFMT(si)
    planner.setNumSamples(500)
    ss.setPlanner(planner)

    solved = ss.solve(2.0)
    assert solved, "BFMT should find a solution"


def test_aorrtc():
    ss, si = create_simple_setup()

    planner = og.AORRTC(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    solved = ss.solve(1.0)
    assert solved, "AORRTC should find a solution"


def test_prmstar():
    ss, si = create_simple_setup()

    planner = og.PRMstar(si)
    ss.setPlanner(planner)

    solved = ss.solve(1.0)
    assert solved, "PRMstar should find a solution"


def test_rrtstar_settings():
    _, si = create_simple_setup()

    planner = og.RRTstar(si)

    # Test setters and getters
    planner.setGoalBias(0.1)
    assert planner.getGoalBias() == pytest.approx(0.1)

    planner.setRange(0.2)
    assert planner.getRange() == pytest.approx(0.2)

    planner.setRewireFactor(1.2)
    assert planner.getRewireFactor() == pytest.approx(1.2)

    planner.setDelayCC(True)
    assert planner.getDelayCC() == True

    planner.setTreePruning(True)
    assert planner.getTreePruning() == True

    planner.setKNearest(True)
    assert planner.getKNearest() == True


def test_bitstar_settings():
    _, si = create_simple_setup()

    planner = og.BITstar(si)

    # Test setters and getters
    planner.setRewireFactor(1.2)
    assert planner.getRewireFactor() == pytest.approx(1.2)

    planner.setSamplesPerBatch(200)
    assert planner.getSamplesPerBatch() == 200

    planner.setUseKNearest(True)
    assert planner.getUseKNearest() == True

    planner.setPruning(True)
    assert planner.getPruning() == True


def test_fmt_settings():
    _, si = create_simple_setup()

    planner = og.FMT(si)

    # Test setters and getters
    planner.setNumSamples(1000)
    assert planner.getNumSamples() == 1000

    planner.setNearestK(True)
    assert planner.getNearestK() == True

    planner.setRadiusMultiplier(1.5)
    assert planner.getRadiusMultiplier() == pytest.approx(1.5)

    planner.setCacheCC(True)
    assert planner.getCacheCC() == True

    planner.setHeuristics(True)
    assert planner.getHeuristics() == True


def test_path_length_optimization():
    ss, si = create_simple_setup()

    planner = og.RRTstar(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    # Run for longer to get better solution
    solved = ss.solve(2.0)
    assert solved, "Should find a solution"

    path = ss.getSolutionPath()
    assert path is not None

    # Check that path length is reasonable
    # Diagonal distance is sqrt(2) * 0.8 ≈ 1.13, but we go around obstacle
    path_length = path.length()
    assert path_length > 1.0, "Path should be longer than straight line due to obstacle"
    assert path_length < 3.0, "Path should not be excessively long"


if __name__ == "__main__":
    test_rrtstar()
    test_informed_rrtstar()
    test_sorrtstar()
    test_bitstar()
    test_fmt()
    test_bfmt()
    test_aorrtc()
    test_prmstar()
    test_rrtstar_settings()
    test_bitstar_settings()
    test_fmt_settings()
    test_path_length_optimization()

