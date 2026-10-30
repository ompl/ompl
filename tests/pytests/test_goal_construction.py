from ompl import base as ob


def _make_space_information():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(lambda _state: True)
    si.setup()
    return si


def test_goal_state_construction():
    si = _make_space_information()
    goal = ob.GoalState(si)

    goal_state = si.allocState()
    goal_state[0] = 0.5
    goal_state[1] = 0.5
    goal.setState(goal_state)

    assert goal.getState() is not None
    assert abs(goal.getState()[0] - 0.5) < 1e-6
    assert abs(goal.getState()[1] - 0.5) < 1e-6


def test_goal_states_construction():
    si = _make_space_information()
    goal_states = ob.GoalStates(si)

    goal_state = si.allocState()
    goal_state[0] = 0.25
    goal_state[1] = -0.25
    goal_states.addState(goal_state)

    assert goal_states.hasStates()
    assert goal_states.getStateCount() == 1
    assert abs(goal_states.getState(0)[0] - 0.25) < 1e-6
    assert abs(goal_states.getState(0)[1] + 0.25) < 1e-6
