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
