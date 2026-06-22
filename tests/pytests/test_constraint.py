import numpy as np
import pytest

from ompl import base as ob


def test_constraint_jacobian_default():
    class UnitSphere(ob.Constraint):
        def __init__(self):
            super().__init__(3, 1)

        def function(self, x, out):
            out[0] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - 1.0

    constraint = UnitSphere()
    assert constraint.getAmbientDimension() == 3
    assert constraint.getCoDimension() == 1

    x = np.array([1.0, 0.0, 0.0])
    # the output array is accepted regardless of memory layout (C- or F-order)
    out = np.zeros((1, 3))
    # jacobian is not overridden, so it falls through to the C++ default (numerical
    # differentiation); the gradient of x^2 + y^2 + z^2 - 1 at (1, 0, 0) is [2, 0, 0]
    constraint.jacobian(x, out)

    assert out[0, 0] == pytest.approx(2.0, abs=1e-4)
    assert out[0, 1] == pytest.approx(0.0, abs=1e-4)
    assert out[0, 2] == pytest.approx(0.0, abs=1e-4)


if __name__ == "__main__":
    test_constraint_jacobian_default()
