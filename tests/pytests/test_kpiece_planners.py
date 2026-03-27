import pytest
from ompl import base as ob
from ompl import geometric as og


def create_simple_setup():
    """Create a simple 2D planning problem for KPIECE planners."""
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

    return ss, si


def test_kpiece1():
    ss, si = create_simple_setup()

    planner = og.KPIECE1(si)
    planner.setRange(0.1)
    planner.setGoalBias(0.05)
    ss.setPlanner(planner)

    solved = ss.solve(2.0)
    assert solved, "KPIECE1 should find a solution"


def test_bkpiece1():
    ss, si = create_simple_setup()

    planner = og.BKPIECE1(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    solved = ss.solve(2.0)
    assert solved, "BKPIECE1 should find a solution"


def test_lbkpiece1():
    ss, si = create_simple_setup()

    planner = og.LBKPIECE1(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    solved = ss.solve(2.0)
    assert solved, "LBKPIECE1 should find a solution"


def test_kpiece1_settings():
    _, si = create_simple_setup()

    planner = og.KPIECE1(si)

    # Test setters and getters
    planner.setGoalBias(0.1)
    assert planner.getGoalBias() == pytest.approx(0.1)

    planner.setRange(0.2)
    assert planner.getRange() == pytest.approx(0.2)

    planner.setBorderFraction(0.8)
    assert planner.getBorderFraction() == pytest.approx(0.8)

    planner.setMinValidPathFraction(0.3)
    assert planner.getMinValidPathFraction() == pytest.approx(0.3)

    planner.setFailedExpansionCellScoreFactor(0.4)
    assert planner.getFailedExpansionCellScoreFactor() == pytest.approx(0.4)


def test_bkpiece1_settings():
    _, si = create_simple_setup()

    planner = og.BKPIECE1(si)

    # Test setters and getters
    planner.setRange(0.2)
    assert planner.getRange() == pytest.approx(0.2)

    planner.setBorderFraction(0.8)
    assert planner.getBorderFraction() == pytest.approx(0.8)

    planner.setMinValidPathFraction(0.3)
    assert planner.getMinValidPathFraction() == pytest.approx(0.3)

    planner.setFailedExpansionCellScoreFactor(0.4)
    assert planner.getFailedExpansionCellScoreFactor() == pytest.approx(0.4)


def test_lbkpiece1_settings():
    _, si = create_simple_setup()

    planner = og.LBKPIECE1(si)

    # Test setters and getters
    planner.setRange(0.2)
    assert planner.getRange() == pytest.approx(0.2)

    planner.setBorderFraction(0.8)
    assert planner.getBorderFraction() == pytest.approx(0.8)

    planner.setMinValidPathFraction(0.3)
    assert planner.getMinValidPathFraction() == pytest.approx(0.3)


def test_kpiece1_clear():
    ss, si = create_simple_setup()

    planner = og.KPIECE1(si)
    planner.setRange(0.1)
    ss.setPlanner(planner)

    # Solve once
    solved = ss.solve(1.0)

    # Clear and solve again
    planner.clear()
    ss.clear()

    start = si.allocState()
    start[0] = 0.1
    start[1] = 0.1

    goal = si.allocState()
    goal[0] = 0.9
    goal[1] = 0.9

    ss.setStartAndGoalStates(start, goal)
    solved = ss.solve(1.0)
    assert solved, "KPIECE1 should find a solution after clear"


def test_discretization():
    """Test Discretization class bindings."""
    # Create a discretization (uses default no-op free function)
    disc = og.Discretization()

    # Test border fraction
    disc.setBorderFraction(0.8)
    assert disc.getBorderFraction() == pytest.approx(0.8)

    # Test dimension
    disc.setDimension(2)

    # Test counts (should be 0 initially)
    assert disc.getMotionCount() == 0
    assert disc.getCellCount() == 0

    # Test clear
    disc.clear()
    assert disc.getMotionCount() == 0

    # Test count iteration
    disc.countIteration()


def test_discretization_cell_data():
    """Test DiscretizationCellData bindings."""
    cell_data = og.DiscretizationCellData()

    # Check default values
    assert cell_data.coverage == pytest.approx(0.0)
    assert cell_data.selections == 1
    assert cell_data.score == pytest.approx(1.0)
    assert cell_data.iteration == 0
    assert cell_data.importance == pytest.approx(0.0)


if __name__ == "__main__":
    test_kpiece1()
    test_bkpiece1()
    test_lbkpiece1()
    test_kpiece1_settings()
    test_bkpiece1_settings()
    test_lbkpiece1_settings()
    test_kpiece1_clear()
    test_discretization()
    test_discretization_cell_data()



