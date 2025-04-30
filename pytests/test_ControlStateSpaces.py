import sys
import pytest

from ompl import base as ob
from ompl import control as oc

def test_control_state_space():
    # Create a control state space
    space = ob.SE2StateSpace()

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a control space
    cspace = oc.RealVectorControlSpace(space, 2)

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-.3)
    cbounds.setHigh(.3)
    cspace.setBounds(cbounds)

    control = cspace.allocControl()
    rvControlSampler = oc.RealVectorControlUniformSampler(cspace)
    rvControlSampler.sample(control)
    cspace.printControl(control)
    control[0] = 0.5
    control[1] = 0.2
    assert control[0] == pytest.approx(0.5)
    assert control[1] == pytest.approx(0.2)

    cspace.printControl(control)

    nullControl = cspace.allocControl()
    cspace.nullControl(nullControl)
    cspace.printControl(nullControl)
    assert nullControl[0] == pytest.approx(0)
    assert nullControl[1] == pytest.approx(0)

    assert cspace.getDimension() == 2
    cspace.printSettings()

def test_control_space_information():
    def propagate(temp1, control, duration, state):
        state.setX(temp1.getX() + control[0] * duration)
        state.setY(temp1.getY() + control[1] * duration)
        state.setYaw(temp1.getYaw())

    space = ob.SE2StateSpace()

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a control space
    cspace = oc.RealVectorControlSpace(space, 2)

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-.3)
    cbounds.setHigh(.3)
    cspace.setBounds(cbounds)  

    si = oc.SpaceInformation(space, cspace)
    si.setPropagationStepSize(1)
    si.setStatePropagator(propagate)

    stateSampler = space.allocDefaultStateSampler()

    # create a random state and a control
    state = space.allocState()
    state.setX(0)
    state.setY(0)
    state.setYaw(0)
    space.printState(state)
    control = cspace.allocControl()
    control[0] = 0.5
    control[1] = 0.2

    newState = space.allocState()
    si.propagate(state, control, 1, newState)

    assert newState.getX() == pytest.approx(0+control[0])
    assert newState.getY() == pytest.approx(0+control[1])
    assert newState.getYaw() == pytest.approx(0)

def test_path_control():
    def propagate(temp1, control, duration, state):
        state.setX(temp1.getX() + control[0] * duration)
        state.setY(temp1.getY() + control[0] * duration)
        state.setYaw(temp1.getYaw() + control[1] * duration)
    space = ob.SE2StateSpace()

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a control space
    cspace = oc.RealVectorControlSpace(space, 2)

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-.3)
    cbounds.setHigh(.3)
    cspace.setBounds(cbounds)  

    si = oc.SpaceInformation(space, cspace)
    si.setStatePropagator(propagate)
    si.setPropagationStepSize(1)

    pc = oc.PathControl(si)

    state = space.allocState()
    stateSampler = space.allocDefaultStateSampler()
    stateSampler.sampleUniform(state)
    space.printState(state)

    control = cspace.allocControl()
    controlSampler = oc.RealVectorControlUniformSampler(cspace)
    controlSampler.sample(control)
    cspace.nullControl(control)

    pc.append(state, control, 1)
    pc.append(state)

    # Print the path
    pc.print()

    assert pc.getStateCount() == 2
    assert pc.getControlCount() == 1

