from math import sin, cos
from functools import partial
from ompl import base as ob
from ompl import control as oc
import pytest

def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are
    # satisfied
    return spaceInformation.satisfiesBounds(state)

def propagate(temp1, control, duration, state):
    state.setX(temp1.getX() + control[0] * duration * cos(temp1.getYaw()))
    state.setY(temp1.getY() + control[0] * duration * sin(temp1.getYaw()))
    state.setYaw(temp1.getYaw() + control[1] * duration)

def plan():
    # construct the state space we are planning in
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
    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    ss.setStateValidityChecker(
        partial(isStateValid, ss.getSpaceInformation()))
        
    ss.setStatePropagator(propagate)

    # create a start state
    start = space.allocState()
    start.setX(-0.5)
    start.setY(0.0)
    start.setYaw(0.0)

    # create a goal state
    goal = space.allocState()
    goal.setX(0.0)
    goal.setY(0.5)
    goal.setYaw(0.0)

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    # (optionally) set planner
    # si = ss.getSpaceInformation()
    # planner = oc.RRT(si)
    # ss.setPlanner(planner)
    # (optionally) set propagation step size
    # si.setPropagationStepSize(.1)

    # attempt to solve the problem
    solved = ss.solve(20.0)

    if solved:
        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath().printAsMatrix())

if __name__ == "__main__":
    plan()