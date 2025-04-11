import sys
import pytest
import math
# Make sure the OMPL bindings are in the Python path.
from ompl import base as ob



def tes_rv_state_space():
    # --- Part 1: Test RealVectorStateSpace (2D) ---
    # Create a 2-dimensional RealVectorStateSpace.
    rvss = ob.RealVectorStateSpace(2)
    assert rvss.getDimension() == 2, "Dimension should be 2"

    # Set up bounds.
    rvBound = ob.RealVectorBounds(2)
    rvBound.setLow(-1)
    rvBound.setHigh(1)
    rvss.setBounds(rvBound)
    
    # Print settings (to see output, run with -s).
    rvss.printSettings()
    
    # Allocate a state and verify its type.
    state = rvss.allocState()
    assert "RealVectorStateType" in str(type(state)), "State is not of type RealVectorStateType"
    
    # Use the state sampler to generate a random state.
    sampler = ob.RealVectorStateSampler(rvss)
    sampler.sampleUniform(state)
    rvss.printState(state)  # This output will be captured unless using -s

    # Manually set state values and verify them.
    state[0] = 0.5
    state[1] = 0.5
    rvss.printState(state)
    assert state[0] == pytest.approx(0.5)
    assert state[1] == pytest.approx(0.5)
    
    # Allocate another state, sample uniformly, and check the distance.
    state_another = rvss.allocState()
    sampler.sampleUniform(state_another)
    rvss.printState(state_another)
    dist = rvss.distance(state, state_another)
    assert isinstance(dist, float), "Distance should be a float value"

    # --- Part 2: Test RealVectorStateSpace ScopedState (10D) ---
    dim = 10
    rvss10 = ob.RealVectorStateSpace(dim)
    rvBound10 = ob.RealVectorBounds(dim)
    rvBound10.setLow(-1)
    rvBound10.setHigh(1)
    rvss10.setBounds(rvBound10)
    
    # Create a ScopedState (using the "rv" submodule binding).
    scopedState = ob.RealVector.ScopedState(rvss10)
    # Check that the type is correct.
    assert "ScopedState" in str(type(scopedState)), "ScopedState type not as expected"
    
    # Set to a random state and print.
    scopedState.random()
    scopedState.print()
    
    # Set each component to 0.5 and verify.
    for i in range(dim):
        scopedState[i] = 0.5
        assert scopedState[i] == pytest.approx(0.5)
    scopedState.print()


def test_se2_state_space():
    # --- Create and setup SE2StateSpace ---
    se2ss = ob.SE2StateSpace()
    # For SE2, the x and y bounds are given via a 2D RealVectorBounds.
    se2Bound = ob.RealVectorBounds(2)
    se2Bound.setLow(-1)
    se2Bound.setHigh(1)
    se2ss.setBounds(se2Bound)
    
    # Optionally, print the space settings (run pytest with -s to see printed output).
    se2ss.printSettings()

    # --- Test ScopedState ---
    # Create a ScopedState using the se2 submodule binding.
    scopedState = ob.SE2.ScopedState(se2ss)
    # Verify that the type name contains "ScopedState"
    assert "ScopedState" in str(type(scopedState)), "SE2 ScopedState type not as expected"
    
    # Randomize the state and print it.
    scopedState.random()
    scopedState.print()

    # Now set the state explicitly:
    # Here we use functions that you have bound in your se2 ScopedState binding.
    # For example, setXY sets the x and y values, and setYaw sets the orientation.
    scopedState.setXY(0.5, 0.5)
    scopedState.setYaw(0)
    scopedState.print()

    # Use the getter methods to retrieve the values.
    x = scopedState.getX()
    y = scopedState.getY()
    yaw = scopedState.getYaw()
    assert x == pytest.approx(0.5), f"Expected getX() to return 0.5, got {x}"
    assert y == pytest.approx(0.5), f"Expected getY() to return 0.5, got {y}"
    assert yaw == pytest.approx(0), f"Expected getYaw() to return 0, got {yaw}"

    # --- Test state copying and distance ---
    # Allocate another state from the state space.
    state_copy = se2ss.allocState()
    # Copy the scoped state's underlying state to the new state.
    se2ss.copyState(state_copy, scopedState.get())
    # Compute the distance between the original state and its copy.
    distance = se2ss.distance(scopedState.get(), state_copy)
    # Since they are identical, the distance should be approximately zero.
    assert distance == pytest.approx(0.0), f"Expected distance 0, got {distance}"


def test_discrete_state_space():
    # --- Test the DiscreteStateSpace itself ---
    # Create a discrete state space with lower bound 0 and upper bound 9.
    dss = ob.DiscreteStateSpace(0, 9)
    # The total number of states should be (upperBound - lowerBound + 1) = 10.
    assert dss.getStateCount() == 10, "Expected 10 states in the discrete space"
    
    # For debugging, print the state space settings.
    dss.printSettings()
    
    # Allocate two states, manually set their 'value' fields, and test distance.
    state1 = dss.allocState()
    state2 = dss.allocState()
    state1.value = 3
    state2.value = 7
    assert state1.value == 3, "State1 value should be 3"
    assert state2.value == 7, "State2 value should be 7"
    
    # Print states (for visual inspection).
    dss.printState(state1)
    dss.printState(state2)
    
    # Compute the distance (for discrete space, usually the absolute difference).
    dist = dss.distance(state1, state2)
    assert dist == pytest.approx(4), f"Expected distance 4, got {dist}"
    
    # Test copying a state: a copied state should have zero distance.
    state_copy = dss.allocState()
    dss.copyState(state_copy, state1)
    copy_dist = dss.distance(state1, state_copy)
    assert copy_dist == pytest.approx(0), f"Expected distance 0, got {copy_dist}"
    
    # --- Test the DiscreteStateSampler methods ---
    sampler = ob.DiscreteStateSampler(dss)
    
    # Use sampleUniform and verify that the sampled value is in the correct range.
    sampler.sampleUniform(state1)
    assert 0 <= state1.value <= 9, f"Uniform sample value {state1.value} out of bounds"
    
    # Use sampleUniformNear; here, we use state2 as a 'near' state.
    sampler.sampleUniformNear(state1, state2, 1.0)
    assert 0 <= state1.value <= 9, f"UniformNear sample value {state1.value} out of bounds"
    
    # Use sampleGaussian; state2 as the mean, with stdDev 0.5.
    sampler.sampleGaussian(state1, state2, 0.5)
    assert 0 <= state1.value <= 9, f"Gaussian sample value {state1.value} out of bounds"
    
    # --- Test the ScopedState binding from the discrete submodule ---
    # Create a ScopedState using the discrete binding.
    scoped_state = ob.Discrete.ScopedState(dss)
    # Check that the type string contains "ScopedState"
    assert "ScopedState" in str(type(scoped_state)), "ScopedState type not as expected"
    
    # Manually set the state value via the property and verify.
    scoped_state.value = 5
    assert scoped_state.value == 5, "Scoped state value should be 5"
    
    # Randomize the state and ensure the sampled value lies within the discrete bounds.
    scoped_state.random()
    random_val = scoped_state.value
    assert 0 <= random_val <= 9, f"Randomly sampled state value {random_val} is out of bounds"
    
    # For visual inspection, print the scoped state.
    scoped_state.print()

    


def test_so2_state_space():
    # --- Test the SO2StateSpace (state space functions) ---
    so2ss = ob.SO2StateSpace()
    # Optionally print settings (visible if running with -s)
    so2ss.printSettings()
    
    # In OMPL, the dimension for SO2 is typically 1.
    assert so2ss.getDimension() == 1, "SO2StateSpace should have dimension 1"
    
    # Allocate a raw state and test its behavior.
    state = so2ss.allocState()
    # Set identity: this should set the angle to 0.
    state.setIdentity()
    assert state.value == pytest.approx(0.0), "setIdentity should set state.value to 0"
    
    # Use the sampler to generate a state.
    sampler = ob.SO2StateSampler(so2ss)
    sampler.sampleUniform(state)
    # Check that the sampled state value is a float and finite.
    assert isinstance(state.value, float), "State value should be a float after sampling"
    assert math.isfinite(state.value), "Sampled state value should be finite"
    
    # Test distance: create another state, set explicit values, and compute distance.
    state2 = so2ss.allocState()
    # Manually set angles (in radians); here, choose two values that do not wrap around.
    state.value = 0.5
    state2.value = 1.2
    dist = so2ss.distance(state, state2)
    # Expect distance to be roughly the absolute difference (0.7 in this case)
    assert dist == pytest.approx(0.7, rel=1e-2), f"Expected distance about 0.7, got {dist}"
    
    # --- Test the ScopedState binding ---
    # Create a ScopedState using the "so2" submodule binding.
    scoped_state = ob.SO2.ScopedState(so2ss)
    # Verify that the type string contains "ScopedState".
    assert "ScopedState" in str(type(scoped_state)), "ScopedState type not as expected"
    
    # Randomize the ScopedState and print it.
    scoped_state.random()
    scoped_state.print()
    
    # Test the property "value" exposed via def_property_rw.
    # First, check that the current value is a float.
    val_before = scoped_state.value
    assert isinstance(val_before, float), "ScopedState's value should be a float"
    
    # Set the value to a known angle.
    new_angle = 1.234
    scoped_state.value = new_angle
    # Verify via the property.
    assert scoped_state.value == pytest.approx(new_angle), "ScopedState 'value' property did not update correctly"
    
    # Use the additional method if bound (for example, setIdentity)
    scoped_state.setIdentity()
    assert scoped_state.value == pytest.approx(0.0), "After setIdentity, value should be 0"
    
    # Finally, print the scoped state for visual inspection.
    scoped_state.print()

def test_so3_state_space():
    # --- Test the SO3StateSpace raw state API ---
    so3ss = ob.SO3StateSpace()
    
    # Optionally, print the settings for debugging.
    so3ss.printSettings()
    
    # Allocate a raw state from the state space.
    state = so3ss.allocState()
    
    # Use the SO3State binding to check quaternion components.
    # Initially, the state might contain arbitrary values.
    # Let’s use the sampler to generate a state.
    sampler = ob.SO3StateSampler(so3ss)
    sampler.sampleUniform(state)
    
    # Access the quaternion components via the raw state.
    x = state.x
    y = state.y
    z = state.z
    w = state.w
    # Verify they are floats.
    assert isinstance(x, float), "x component should be a float"
    assert isinstance(y, float), "y component should be a float"
    assert isinstance(z, float), "z component should be a float"
    assert isinstance(w, float), "w component should be a float"
    
    # Test the norm function: norm() should return approximately 1 for a normalized quaternion.
    n = so3ss.norm(state)
    assert n == pytest.approx(1.0, rel=1e-2), f"Expected norm to be ~1.0, got {n}"
    
    # Allocate a second state and set it to a different quaternion.
    state2 = so3ss.allocState()
    state2.x = 0.1
    state2.y = 0.2
    state2.z = 0.3
    state2.w = math.sqrt(1 - (0.1**2 + 0.2**2 + 0.3**2))
    so3ss.printState(state)
    so3ss.printState(state2)
    # Compute the distance between state and state2.
    d = so3ss.distance(state, state2)
    assert d >= 0.0, "Distance should be non-negative"

    # Test copyState: copy state2 into a new state and check that distance is zero.
    state_copy = so3ss.allocState()
    so3ss.copyState(state_copy, state2)
    d_copy = so3ss.distance(state2, state_copy)
    assert d_copy == pytest.approx(0.0), f"Expected copy distance 0.0, got {d_copy}"
    
    # --- Test the ScopedState binding from the so3 submodule ---
    scoped_state = ob.SO3.ScopedState(so3ss)
    # Check that the type string contains "ScopedState"
    assert "ScopedState" in str(type(scoped_state)), "ScopedState type not as expected"
    
    # Randomize the scoped state and print it.
    scoped_state.random()
    print("Randomized ScopedState:")
    scoped_state.print()
    
    # Test the property bindings: set and read quaternion components.
    scoped_state.x = 0.0
    scoped_state.y = 0.0
    scoped_state.z = 0.0
    scoped_state.w = 1.0
    assert scoped_state.x == pytest.approx(0.0), f"Expected x to be 0.0, got {scoped_state.x}"
    assert scoped_state.y == pytest.approx(0.0), f"Expected y to be 0.0, got {scoped_state.y}"
    assert scoped_state.z == pytest.approx(0.0), f"Expected z to be 0.0, got {scoped_state.z}"
    assert scoped_state.w == pytest.approx(1.0), f"Expected w to be 1.0, got {scoped_state.w}"
    
    # Print final scoped state.
    scoped_state.print()


def test_se3_state_space():
    # ==== Test the SE3StateSpace raw API ====
    # Create an SE3StateSpace.
    se3ss = ob.SE3StateSpace()
    
    # Create bounds for the translational (RealVector) component (3D bounds).
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-1)
    bounds.setHigh(1)
    se3ss.setBounds(bounds)
    
    # Retrieve the bounds and, if supported, verify the limits.
    # (The API may expose lower and upper bounds as lists or objects.)
    b = se3ss.getBounds()
    # Here, we assume b.getLow() and b.getHigh() return sequences of 3 elements.
    lb = b.low
    ub = b.high
    assert lb[0] == pytest.approx(-1), "Lower bound of first component should be -1"
    assert ub[0] == pytest.approx(1), "Upper bound of first component should be 1"
    
    # Allocate a raw state.
    raw_state = se3ss.allocState()
    
    # Set translational components using the bound setter functions.
    raw_state.setXYZ(0.1, 0.2, 0.3)
    # For the rotational part, access the rotation via the state’s 'rotation()' method
    # and set it to identity.
    raw_state.rotation().setIdentity()
    
    # Verify the translational values using getters.
    x_val = raw_state.getX()
    y_val = raw_state.getY()
    z_val = raw_state.getZ()
    assert x_val == pytest.approx(0.1), f"Expected x ~0.1, got {x_val}"
    assert y_val == pytest.approx(0.2), f"Expected y ~0.2, got {y_val}"
    assert z_val == pytest.approx(0.3), f"Expected z ~0.3, got {z_val}"
    
    # Optionally, print the raw state.
    se3ss.printState(raw_state)
    
    # Call registerProjections (if the function does something meaningful).
    se3ss.registerProjections()
    
    # ==== Test the ScopedState binding ====
    # Create a ScopedState from the SE3 submodule binding.
    scoped_state = ob.SE3.ScopedState(se3ss)
    
    # Verify that the type includes "ScopedState" in its name.
    assert "ScopedState" in str(type(scoped_state)), "ScopedState type not as expected"
    
    # Randomize the scoped state and print.
    scoped_state.random()
    scoped_state.print()
    
    # Now explicitly set the translational components via the convenience methods.
    scoped_state.setXYZ(0.5, 0.6, 0.7)
    x_scoped = scoped_state.getX()
    y_scoped = scoped_state.getY()
    z_scoped = scoped_state.getZ()
    assert x_scoped == pytest.approx(0.5), f"Expected scoped x ~0.5, got {x_scoped}"
    assert y_scoped == pytest.approx(0.6), f"Expected scoped y ~0.6, got {y_scoped}"
    assert z_scoped == pytest.approx(0.7), f"Expected scoped z ~0.7, got {z_scoped}"
    
    # Optionally, print the final scoped state.
    scoped_state.print()

def test_time_state_space():
    # === Part 1: Raw TimeStateSpace API ===
    # Create a TimeStateSpace instance.
    timeSS = ob.TimeStateSpace()
    
    # Set bounds for the time state space (for example, time in [0.0, 10.0]).
    timeSS.setBounds(0.0, 10.0)
    
    # Verify that the bounds were set correctly.
    assert timeSS.getMinTimeBound() == pytest.approx(0.0), "Minimum time bound should be 0.0"
    assert timeSS.getMaxTimeBound() == pytest.approx(10.0), "Maximum time bound should be 10.0"
    # Typically a time state space is considered bounded if setBounds has been called.
    assert timeSS.isBounded() == True, "TimeStateSpace should be bounded after setBounds()"
    
    # Print the settings (requires running pytest with -s).
    timeSS.printSettings()
    
    # Allocate a raw state and manually set its position.
    raw_state = timeSS.allocState()
    # Set a valid time value.
    raw_state.position = 5.0
    # Enforce bounds, if necessary.
    timeSS.enforceBounds(raw_state)
    assert raw_state.position >= 0.0 and raw_state.position <= 10.0, "State position must be within bounds"
    
    # Print the raw state for visual inspection.
    timeSS.printState(raw_state)
    
    # Test distance: create a second state.
    raw_state2 = timeSS.allocState()
    raw_state2.position = 7.5
    timeSS.enforceBounds(raw_state2)
    # The distance in time is expected to be the absolute difference.
    dist = timeSS.distance(raw_state, raw_state2)
    assert dist == pytest.approx(2.5), f"Expected distance 2.5, got {dist}"
    
    # Test copyState: copy raw_state into raw_state2 and then distance should be 0.
    timeSS.copyState(raw_state2, raw_state)
    copy_dist = timeSS.distance(raw_state, raw_state2)
    assert copy_dist == pytest.approx(0.0), f"Expected copied state distance 0.0, got {copy_dist}"
    
    # Test the TimeStateSampler.
    sampler = ob.TimeStateSampler(timeSS)
    sampled_state = timeSS.allocState()
    sampler.sampleUniform(sampled_state)
    # Check that the sampled time is within bounds.
    assert 0.0 <= sampled_state.position <= 10.0, f"Uniform sampled time {sampled_state.position} out of bounds"
    
    # === Part 2: ScopedState Binding (from the "time" submodule) ===
    # Create a ScopedState using the binding in the "time" submodule.
    scoped_state = ob.Time.ScopedState(timeSS)
    
    # Verify that the type contains "ScopedState".
    assert "ScopedState" in str(type(scoped_state)), "ScopedState type not as expected"
    
    # Randomize the scoped state and print.
    scoped_state.random()
    scoped_state.print()
    
    # Test the property binding: set the time and then verify.
    scoped_state.position = 3.1415
    assert scoped_state.position == pytest.approx(3.1415), f"Expected position 3.1415, got {scoped_state.position}"
    
    # Optionally, check consistency by enforcing bounds.
    timeSS.enforceBounds(scoped_state.get())
    assert 0.0 <= scoped_state.position <= 10.0, "Scoped state position must be within bounds"
    
    # Print the final scoped state.
    scoped_state.print()
