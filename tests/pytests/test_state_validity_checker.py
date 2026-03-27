import pytest
from ompl import base as ob


class MyStateValidityChecker(ob.StateValidityChecker):
    def __init__(self, si: ob.SpaceInformation):
        super().__init__(si)
        self.si = si

    def isValid(self, state: ob.State) -> bool:
        # Simple checker: valid if first component is in [-0.5, 0.5]
        return -0.5 <= state[0] <= 0.5


class ClearanceStateValidityChecker(ob.StateValidityChecker):
    def __init__(self, si: ob.SpaceInformation):
        super().__init__(si)
        self.si = si

    def isValid(self, state: ob.State) -> bool:
        # Valid if within unit circle centered at origin
        x, y = state[0], state[1]
        return x * x + y * y <= 1.0

    def clearance(self, state: ob.State) -> float:
        # Distance to boundary of unit circle
        import math
        x, y = state[0], state[1]
        dist_from_center = math.sqrt(x * x + y * y)
        return 1.0 - dist_from_center


def create_space_and_si():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)
    si = ob.SpaceInformation(space)
    return space, si


def test_custom_state_validity_checker():
    space, si = create_space_and_si()

    checker = MyStateValidityChecker(si)
    si.setStateValidityChecker(checker)
    si.setup()

    # Test valid state
    valid_state = si.allocState()
    valid_state[0] = 0.0
    valid_state[1] = 0.0
    assert checker.isValid(valid_state), "State at origin should be valid"

    # Test invalid state
    invalid_state = si.allocState()
    invalid_state[0] = 0.8
    invalid_state[1] = 0.0
    assert not checker.isValid(invalid_state), "State at 0.8 should be invalid"

    # Test boundary
    boundary_state = si.allocState()
    boundary_state[0] = 0.5
    boundary_state[1] = 0.0
    assert checker.isValid(boundary_state), "State at 0.5 should be valid"


def test_clearance_state_validity_checker():
    space, si = create_space_and_si()

    checker = ClearanceStateValidityChecker(si)
    si.setStateValidityChecker(checker)
    si.setup()

    # Test state at origin - maximum clearance
    center_state = si.allocState()
    center_state[0] = 0.0
    center_state[1] = 0.0
    assert checker.isValid(center_state)
    assert checker.clearance(center_state) == pytest.approx(1.0)

    # Test state at boundary - zero clearance
    boundary_state = si.allocState()
    boundary_state[0] = 1.0
    boundary_state[1] = 0.0
    assert checker.isValid(boundary_state)
    assert checker.clearance(boundary_state) == pytest.approx(0.0)

    # Test state outside - negative clearance
    outside_state = si.allocState()
    outside_state[0] = 1.5
    outside_state[1] = 0.0
    assert not checker.isValid(outside_state)
    assert checker.clearance(outside_state) == pytest.approx(-0.5)


def test_all_valid_state_validity_checker():
    space, si = create_space_and_si()

    checker = ob.AllValidStateValidityChecker(si)
    si.setStateValidityChecker(checker)
    si.setup()

    # Any state should be valid
    state = si.allocState()
    state[0] = 0.99
    state[1] = -0.99
    assert checker.isValid(state), "AllValidStateValidityChecker should accept any state"


def test_state_validity_checker_specs():
    specs = ob.StateValidityCheckerSpecs()

    # Test default values
    assert specs.clearanceComputationType == ob.ClearanceComputationType.NONE
    assert specs.hasValidDirectionComputation == False

    # Test setting values
    specs.clearanceComputationType = ob.ClearanceComputationType.EXACT
    specs.hasValidDirectionComputation = True
    assert specs.clearanceComputationType == ob.ClearanceComputationType.EXACT
    assert specs.hasValidDirectionComputation == True



def test_lambda_state_validity_checker():
    space, si = create_space_and_si()

    # Test lambda-based validity checker
    def is_valid(state):
        return state[0] >= 0

    si.setStateValidityChecker(is_valid)
    si.setup()

    valid_state = si.allocState()
    valid_state[0] = 0.5
    valid_state[1] = 0.0
    assert si.isValid(valid_state)

    invalid_state = si.allocState()
    invalid_state[0] = -0.5
    invalid_state[1] = 0.0
    assert not si.isValid(invalid_state)


if __name__ == "__main__":
    test_custom_state_validity_checker()
    test_clearance_state_validity_checker()
    test_all_valid_state_validity_checker()
    test_state_validity_checker_specs()
    test_lambda_state_validity_checker()

