from __future__ import annotations

import pytest
from ompl import base as ob
from ompl import geometric as og


def make_space() -> ob.RealVectorStateSpace:
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(1)
    space.setBounds(bounds)
    return space


def is_valid(state: ob.State) -> bool:
    # Wall across 0.4 < x < 0.6, with a gap above y = 0.7
    return not (0.4 < state[0] < 0.6 and state[1] < 0.7)


class LastValidMotionValidator(ob.MotionValidator):
    def __init__(self, si: ob.SpaceInformation):
        super().__init__(si)
        self.si = si
        self.scratch = si.allocState()
        self.last_valid_calls = 0

    def walk(
        self, s1: ob.State, s2: ob.State, last_valid: ob.State | None
    ) -> tuple[bool, float]:
        space = self.si.getStateSpace()
        segments = max(space.validSegmentCount(s1, s2), 1)
        for i in range(1, segments):
            space.interpolate(s1, s2, i / segments, self.scratch)
            if not self.si.isValid(self.scratch):
                return self.stop_at(s1, s2, last_valid, (i - 1) / segments)
        if not self.si.isValid(s2):
            return self.stop_at(s1, s2, last_valid, (segments - 1) / segments)
        return True, 1.0

    def stop_at(
        self, s1: ob.State, s2: ob.State, last_valid: ob.State | None, fraction: float
    ) -> tuple[bool, float]:
        if last_valid is not None:
            self.si.getStateSpace().interpolate(s1, s2, fraction, last_valid)
        return False, fraction

    def checkMotion(self, s1: ob.State, s2: ob.State) -> bool:
        return self.walk(s1, s2, None)[0]

    def checkMotionWithLastValid(
        self, s1: ob.State, s2: ob.State, last_valid: ob.State | None
    ) -> tuple[bool, float]:
        self.last_valid_calls += 1
        return self.walk(s1, s2, last_valid)


class PlainMotionValidator(ob.MotionValidator):
    def __init__(self, si: ob.SpaceInformation):
        super().__init__(si)
        self.si = si

    def checkMotion(self, s1: ob.State, s2: ob.State) -> bool:
        return is_valid(s2)


def make_si() -> ob.SpaceInformation:
    si = ob.SpaceInformation(make_space())
    si.setStateValidityChecker(is_valid)
    si.setup()
    return si


def blocked_motion(si: ob.SpaceInformation) -> tuple[ob.State, ob.State]:
    # Straight across the wall at y = 0.5, so the motion parameter equals x
    s1 = si.allocState()
    s1[0], s1[1] = 0.0, 0.5
    s2 = si.allocState()
    s2[0], s2[1] = 1.0, 0.5
    return s1, s2


def test_last_valid_state_and_fraction_reach_cpp():
    si = make_si()
    validator = LastValidMotionValidator(si)
    s1, s2 = blocked_motion(si)
    last_valid = si.allocState()

    valid, fraction = validator.checkMotionWithLastValid(s1, s2, last_valid)

    assert not valid
    assert validator.last_valid_calls == 1
    assert 0.3 < fraction < 0.4
    assert last_valid[0] == pytest.approx(fraction)
    assert last_valid[1] == pytest.approx(0.5)
    assert is_valid(last_valid)


def test_unobstructed_motion_reports_full_fraction():
    si = make_si()
    validator = LastValidMotionValidator(si)
    s1, s2 = blocked_motion(si)
    s2[0] = 0.3
    last_valid = si.allocState()

    assert validator.checkMotionWithLastValid(s1, s2, last_valid) == (True, 1.0)


def test_last_valid_state_may_be_none():
    si = make_si()
    validator = LastValidMotionValidator(si)
    s1, s2 = blocked_motion(si)

    valid, fraction = validator.checkMotionWithLastValid(s1, s2, None)

    assert not valid
    assert 0.3 < fraction < 0.4


def test_validator_without_override_falls_back():
    si = make_si()
    validator = PlainMotionValidator(si)
    s1, s2 = blocked_motion(si)
    s2[0] = 0.5
    last_valid = si.allocState()

    assert validator.checkMotionWithLastValid(s1, s2, last_valid) == (False, 0.0)


def test_planner_reaches_the_override():
    si = make_si()
    validator = LastValidMotionValidator(si)
    si.setMotionValidator(validator)
    start = si.allocState()
    start[0], start[1] = 0.05, 0.05
    goal = si.allocState()
    goal[0], goal[1] = 0.95, 0.95

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)
    planner = og.LBKPIECE1(si)
    planner.setRange(0.2)
    ss.setPlanner(planner)

    assert ss.solve(5.0)
    assert validator.last_valid_calls > 0
    assert ss.getSolutionPath().check()

    # setMotionValidator holds the validator through a shared_ptr the collector cannot see, so the
    # si <-> validator cycle outlives the test unless a C++ validator takes its place.
    si.setMotionValidator(ob.DiscreteMotionValidator(si))
