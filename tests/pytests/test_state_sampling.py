import pytest
from ompl import base as ob
from ompl import util as ou


def create_space_and_si():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)
    si = ob.SpaceInformation(space)
    return space, si


class MyValidStateSampler(ob.ValidStateSampler):
    def __init__(self, si):
        super().__init__(si)
        self.si_ = si
        self.name_ = "my_sampler"
        self.rng_ = ou.RNG()
        self.sample_count = 0

    def sample(self, state):
        self.sample_count += 1
        state[0] = self.rng_.uniformReal(-1, 1)
        state[1] = self.rng_.uniformReal(-1, 1)
        return self.si_.isValid(state)

    def sampleNear(self, state, near, distance):
        self.sample_count += 1
        state[0] = near[0] + self.rng_.uniformReal(-distance, distance)
        state[1] = near[1] + self.rng_.uniformReal(-distance, distance)
        return self.si_.isValid(state)


class DeterministicValidStateSampler(ob.ValidStateSampler):
    def __init__(self, si):
        super().__init__(si)
        self.si_ = si

    def sample(self, state):
        state[0] = 0.25
        state[1] = 0.25
        return True

    def sampleNear(self, state, near, distance):
        state[0] = near[0] + distance * 0.5
        state[1] = near[1] + distance * 0.5
        return True


def test_custom_valid_state_sampler():
    space, si = create_space_and_si()

    def is_valid(state):
        return True

    si.setStateValidityChecker(is_valid)
    si.setup()

    sampler = MyValidStateSampler(si)
    assert sampler.sample_count == 0

    state = si.allocState()
    result = sampler.sample(state)
    assert result == True
    assert sampler.sample_count == 1
    assert -1 <= state[0] <= 1
    assert -1 <= state[1] <= 1


def test_valid_state_sampler_sample_near():
    space, si = create_space_and_si()

    def is_valid(state):
        return True

    si.setStateValidityChecker(is_valid)
    si.setup()

    sampler = DeterministicValidStateSampler(si)

    near_state = si.allocState()
    near_state[0] = 0.0
    near_state[1] = 0.0

    state = si.allocState()
    result = sampler.sampleNear(state, near_state, 0.2)
    assert result == True
    assert state[0] == pytest.approx(0.1)
    assert state[1] == pytest.approx(0.1)


def test_valid_state_sampler_name():
    space, si = create_space_and_si()

    def is_valid(state):
        return True

    si.setStateValidityChecker(is_valid)
    si.setup()

    sampler = MyValidStateSampler(si)
    sampler.setName("test_sampler")
    assert sampler.getName() == "test_sampler"


def test_valid_state_sampler_nr_attempts():
    space, si = create_space_and_si()

    def is_valid(state):
        return True

    si.setStateValidityChecker(is_valid)
    si.setup()

    sampler = MyValidStateSampler(si)
    default_attempts = sampler.getNrAttempts()
    assert default_attempts > 0

    sampler.setNrAttempts(200)
    assert sampler.getNrAttempts() == 200


def test_obstacle_based_valid_state_sampler():
    space, si = create_space_and_si()

    # Create a validity checker with an obstacle in the center
    def is_valid(state):
        x, y = state[0], state[1]
        # Obstacle: square from -0.2 to 0.2
        if -0.2 <= x <= 0.2 and -0.2 <= y <= 0.2:
            return False
        return True

    si.setStateValidityChecker(is_valid)
    si.setup()

    sampler = ob.ObstacleBasedValidStateSampler(si)

    # Sample many states and check they are valid
    state = si.allocState()
    valid_count = 0
    for _ in range(50):
        if sampler.sample(state):
            valid_count += 1
            assert si.isValid(state)

    # We should get at least some valid samples
    assert valid_count > 0


def test_obstacle_based_valid_state_sampler_sample_near():
    space, si = create_space_and_si()

    def is_valid(state):
        return True

    si.setStateValidityChecker(is_valid)
    si.setup()

    sampler = ob.ObstacleBasedValidStateSampler(si)

    near_state = si.allocState()
    near_state[0] = 0.5
    near_state[1] = 0.5

    state = si.allocState()
    result = sampler.sampleNear(state, near_state, 0.1)
    # ObstacleBasedValidStateSampler's sampleNear should work
    assert isinstance(result, bool)


def test_valid_state_sampler_allocator():
    space, si = create_space_and_si()

    def is_valid(state):
        return True

    si.setStateValidityChecker(is_valid)

    def my_allocator(si_ptr):
        return MyValidStateSampler(si_ptr)

    si.setValidStateSamplerAllocator(my_allocator)
    si.setup()

    # Get a valid state sampler through SpaceInformation
    sampler = si.allocValidStateSampler()
    assert sampler is not None

    state = si.allocState()
    result = sampler.sample(state)
    assert result == True


if __name__ == "__main__":
    test_custom_valid_state_sampler()
    test_valid_state_sampler_sample_near()
    test_valid_state_sampler_name()
    test_valid_state_sampler_nr_attempts()
    test_obstacle_based_valid_state_sampler()
    test_obstacle_based_valid_state_sampler_sample_near()
    test_valid_state_sampler_allocator()
    print("All tests passed!")
