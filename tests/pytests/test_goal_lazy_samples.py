from ompl import base as ob


def test_goal_lazy_samples_add_state():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    def is_valid(_state):
        return True

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    sample_count = 0

    def sampler(_gls, state):
        nonlocal sample_count
        sample_count += 1
        state[0] = 0.4
        state[1] = 0.4
        return sample_count < 1

    goal = ob.GoalLazySamples(si, sampler, autoStart=False)
    assert not goal.isSampling()

    goal_state = si.allocState()
    goal_state[0] = 0.5
    goal_state[1] = 0.5
    goal.addState(goal_state)
    assert goal.hasStates()
    assert goal.getStateCount() == 1

    sampled = si.allocState()
    goal.sampleGoal(sampled)
    assert abs(sampled[0] - 0.5) < 1e-6
    assert abs(sampled[1] - 0.5) < 1e-6

    dist = goal.distanceGoal(sampled)
    assert dist < 1e-6


def test_goal_lazy_samples_callback():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(lambda _state: True)
    si.setup()

    seen = []

    def sampler(_gls, state):
        state[0] = 0.0
        state[1] = 0.0
        return False

    goal = ob.GoalLazySamples(si, sampler, autoStart=False)
    goal.setNewStateCallback(lambda _state: seen.append(True))

    state = si.allocState()
    state[0] = 0.1
    state[1] = 0.1
    assert goal.addStateIfDifferent(state, 0.05)
    assert seen
