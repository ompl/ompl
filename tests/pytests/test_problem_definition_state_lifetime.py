from ompl import base as ob


def test_problem_definition_start_state_lifetime():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(lambda _state: True)
    si.setup()

    pdef = ob.ProblemDefinition(si)
    start = si.allocState()
    start[0] = -0.5
    start[1] = -0.5
    pdef.addStartState(start)

    cached_start = pdef.getStartState(0)
    assert cached_start[0] == -0.5
    assert cached_start[1] == -0.5

    input_states = pdef.getInputStates()
    assert len(input_states) >= 1
    assert input_states[0][0] == -0.5

    goal = si.allocState()
    goal[0] = 0.5
    goal[1] = 0.5
    pdef.setGoalState(goal)
    assert pdef.getGoal() is not None
