import gc

import numpy as np
import pytest

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou


def plane(order=2):
    """A flat state space over the unit square, with every derivative level bounded by 2."""
    output = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1.0)
    bounds.setHigh(1.0)
    output.setBounds(bounds)
    space = ob.FlatStateSpace(output, order)
    for level in range(1, order):
        space.setDerivativeBound(level, 2.0)
    space.setup()
    return space


def test_flat_states_round_trip_through_numpy():
    space = plane(order=3)
    state = space.allocState()

    # Every entry differs, so a transposed or reordered copy can't pass.
    flat = np.arange(6.0).reshape(3, 2) / 10.0
    for layout in (np.ascontiguousarray(flat), np.asfortranarray(flat)):
        space.fromFlatState(layout, state)
        np.testing.assert_array_equal(space.toFlatState(state), flat)

    assert [state.output()[i] for i in range(2)] == pytest.approx(flat[0])
    for level in (1, 2):
        assert [state.derivative(level)[i] for i in range(2)] == pytest.approx(
            flat[level]
        )


def test_plans_a_path_a_trajectory_retraces():
    ou.RNG.setSeed(7)
    space = plane()
    setup = og.SimpleSetup(space)

    # The wall at x of zero leaves a gap above y of 0.4.
    def isStateValid(state):
        p = state.output()
        return space.satisfiesBounds(state) and (abs(p[0]) > 0.1 or p[1] > 0.4)

    setup.setStateValidityChecker(isStateValid)
    setup.setOptimizationObjective(ob.FlatEffortObjective(setup.getSpaceInformation()))

    start = space.allocState()
    goal = space.allocState()
    space.fromFlatState(np.array([[-0.8, -0.8], [0.0, 0.0]]), start)
    space.fromFlatState(np.array([[0.8, -0.8], [0.0, 0.0]]), goal)
    setup.setStartAndGoalStates(start, goal)
    setup.setPlanner(og.RRTConnect(setup.getSpaceInformation()))

    assert setup.solve(5.0) == ob.PlannerStatus.EXACT_SOLUTION
    path = setup.getSolutionPath()
    assert path.check()

    trajectory = ob.FlatTrajectory(path)
    assert len(trajectory) > 0
    state = space.allocState()
    trajectory.toState(trajectory.duration(), state)
    np.testing.assert_allclose(
        space.toFlatState(state), space.toFlatState(goal), atol=1e-9
    )

    # A motion runs in the chart at the state it leaves, so at its duration it has covered the displacement
    # that interpolating along it to the end covers.
    first = trajectory.motion(0)
    space.interpolate(path.getState(0), first, 1.0, state)
    displacement = space.toFlatState(state)[0] - space.toFlatState(path.getState(0))[0]
    np.testing.assert_allclose(
        first.evaluate(first.duration()), displacement, atol=1e-9
    )
    np.testing.assert_allclose(
        first.evaluate(first.duration(), 1), space.toFlatState(state)[1], atol=1e-9
    )


class PythonRealVectorChart(ob.FlatChart):
    def difference(self, from_, to, out):
        for i in range(self.getDimension()):
            out[i] = to[i] - from_[i]

    def advance(self, from_, delta, out):
        for i in range(self.getDimension()):
            out[i] = from_[i] + delta[i]


def test_a_python_chart_steers_like_the_builtin_one():
    output = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1.0)
    bounds.setHigh(1.0)
    output.setBounds(bounds)

    # The space holds the only reference to the chart once it's built.
    space = ob.FlatStateSpace(output, 2, PythonRealVectorChart(2))
    gc.collect()
    space.setDerivativeBound(1, 2.0)
    space.setup()
    reference = plane()

    a, b, mid, expected = (space.allocState() for _ in range(4))
    space.fromFlatState(np.array([[-0.5, 0.2], [1.0, 0.0]]), a)
    space.fromFlatState(np.array([[0.4, -0.3], [0.0, -1.0]]), b)

    space.interpolate(a, b, 0.3, mid)
    reference.interpolate(a, b, 0.3, expected)
    np.testing.assert_allclose(
        space.toFlatState(mid), reference.toFlatState(expected), atol=1e-12
    )
    assert space.steeringCost(a, b) == pytest.approx(reference.steeringCost(a, b))


class StraightLineSteering(ob.FlatSteering):
    """Order 1 steering along a straight line lasting one unit of time."""

    def __init__(self):
        super().__init__(1)

    def steer(self, from_, to):
        return ob.FlatMotion(np.vstack([from_[0], to[0] - from_[0]]), 1.0)


def test_a_python_steering_reaches_the_builtin_cost():
    steering = StraightLineSteering()
    steering.setRho(2.0)
    from_ = np.array([[0.0, 1.0]])
    to = np.array([[3.0, 5.0]])

    # The default steeringCost runs in C++ and calls back into the Python steer through the trampoline.
    motion = steering.steer(from_, to)
    np.testing.assert_allclose(motion.evaluate(1.0), to[0])
    assert steering.steeringCost(from_, to) == pytest.approx(25.0 + 2.0)
    assert steering.cost(motion) == pytest.approx(motion.cost(1, 2.0))
