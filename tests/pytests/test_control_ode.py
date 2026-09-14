from functools import partial
from math import cos, sin, tan

import pytest

from ompl import base as ob
from ompl import control as oc


def kinematic_car_ode(_q, u, qdot):
    theta = _q[2]
    car_length = 0.2
    qdot[0] = u[0] * cos(theta)
    qdot[1] = u[0] * sin(theta)
    qdot[2] = u[0] * tan(u[1]) / car_length


def is_state_valid(space_information, state):
    return space_information.satisfiesBounds(state)


def test_ode_basic_solver_smoke():
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    cspace = oc.RealVectorControlSpace(space, 2)
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-0.3)
    cbounds.setHigh(0.3)
    cspace.setBounds(cbounds)

    ss = oc.SimpleSetup(cspace)
    ss.setStateValidityChecker(
        lambda state: is_state_valid(ss.getSpaceInformation(), state)
    )
    ode = oc.ODE(kinematic_car_ode)
    ode_solver = oc.ODEBasicSolver(ss.getSpaceInformation(), ode)
    ss.setStatePropagator(oc.ODESolver.getStatePropagator(ode_solver))

    start = ss.getStateSpace().allocState()
    start.setX(-0.5)
    start.setY(0.0)
    start.setYaw(0.0)

    goal = ss.getStateSpace().allocState()
    goal.setX(0.0)
    goal.setY(0.5)
    goal.setYaw(0.0)

    ss.setStartAndGoalStates(start, goal, 0.05)
    ss.setup()
    result = ss.solve(2.0)
    assert result


def test_ode_derivative_reaches_integrator():
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-2)
    bounds.setHigh(2)
    space.setBounds(bounds)

    cspace = oc.RealVectorControlSpace(space, 2)
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-1)
    cbounds.setHigh(1)
    cspace.setBounds(cbounds)

    si = oc.SpaceInformation(space, cspace)
    si.setStateValidityChecker(lambda _state: True)
    si.setPropagationStepSize(0.1)
    ode_solver = oc.ODEBasicSolver(si, oc.ODE(kinematic_car_ode))
    si.setStatePropagator(oc.ODESolver.getStatePropagator(ode_solver))
    si.setup()

    start = space.allocState()
    start.setX(0.0)
    start.setY(0.0)
    start.setYaw(0.0)

    control = cspace.allocControl()
    control[0] = 1.0
    control[1] = 0.0

    result = space.allocState()
    si.propagate(start, control, 10, result)

    # yaw stays 0 and u = (1, 0), so qdot = (1, 0, 0) throughout: x advances by the
    # full 1.0s of propagation. If the ODE's out-parameter is dropped on the way back
    # from Python the integrator sees a zero derivative and the state never moves.
    assert result.getX() == pytest.approx(1.0, abs=1e-6)
    assert result.getY() == pytest.approx(0.0, abs=1e-6)
    assert result.getYaw() == pytest.approx(0.0, abs=1e-6)
