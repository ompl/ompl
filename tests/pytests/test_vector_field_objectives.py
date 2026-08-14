import numpy as np
import pytest

from ompl import base as ob


def make_space_information():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(lambda _state: True)
    si.setup()
    return si


def make_states(si):
    s1 = si.allocState()
    s1[0] = -0.5
    s1[1] = 0.0
    s2 = si.allocState()
    s2[0] = 0.5
    s2[1] = 0.0
    return s1, s2


# The vector field returns a numpy array, which only converts to Eigen::VectorXd if the
# binding pulls in nanobind's Eigen caster.
def constant_field(_state):
    return np.array([1.0, 0.0])


def test_upstream_criterion_objective_accepts_numpy_field():
    si = make_space_information()
    objective = ob.VFUpstreamCriterionOptimizationObjective(si, constant_field)
    s1, s2 = make_states(si)

    # Motion runs along +x and the field is a unit vector along +x, so norm - dot is
    # zero at every sample: a field flowing with the motion costs nothing.
    assert objective.motionCost(s1, s2).value() == pytest.approx(0.0, abs=1e-9)


def test_upstream_criterion_objective_penalises_upstream_motion():
    si = make_space_information()
    objective = ob.VFUpstreamCriterionOptimizationObjective(
        si, lambda _s: np.array([-1.0, 0.0])
    )
    s1, s2 = make_states(si)

    # Field points against the motion, so norm - dot is 2 over a path of length 1.
    assert objective.motionCost(s1, s2).value() == pytest.approx(2.0, abs=1e-6)


def test_mechanical_work_objective_accepts_numpy_field():
    si = make_space_information()
    objective = ob.VFMechanicalWorkOptimizationObjective(si, constant_field)
    s1, s2 = make_states(si)

    assert objective.motionCost(s1, s2).value() > 0.0
