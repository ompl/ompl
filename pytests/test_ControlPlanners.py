from math import sin, cos
from functools import partial
from ompl import base as ob
from ompl import control as oc
import pytest

def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are satisfied
    return spaceInformation.satisfiesBounds(state)

def propagate(temp1, control, duration, state):
    # For demonstration, intentionally messing up the partial usage
    # but let's keep it as your snippet.  Real usage would typically do cos/sin.
    state.setX(temp1.getX() + control[0] * duration * (temp1.getYaw()))
    state.setY(temp1.getY() + control[0] * duration * (temp1.getYaw()))
    state.setYaw(temp1.getYaw() + control[1] * duration)

def test_control_no_planner():
    # 1) Construct the SE2 state space
    space = ob.SE2StateSpace()
    
    # set R^2 bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # 2) Create a real-vector control space of dimension=2
    cspace = oc.RealVectorControlSpace(space, 2)
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-0.3)
    cbounds.setHigh(0.3)
    cspace.setBounds(cbounds)

    # 3) Construct a SpaceInformation from that (space, cspace)
    si = oc.SpaceInformation(space, cspace)
    si.setPropagationStepSize(1.0)

    # 4) Build a SimpleSetup from the SpaceInformation
    ss = oc.SimpleSetup(si)

    # 5) Provide a state validity checker as a lambda
    # This partial-lambda structure ensures the argument signature matches (State*) -> bool
    ss.setStateValidityChecker(lambda s: isStateValid(ss.getSpaceInformation(), s))
    
    # 6) Provide a state propagator
    ss.setStatePropagator(propagate)

    # 7) Create start and goal states
    start = space.allocState()
    start.setX(-0.5)
    start.setY(0.0)
    start.setYaw(0.0)

    goal = space.allocState()
    goal.setX(0.0)
    goal.setY(0.5)
    goal.setYaw(0.0)

    # 8) Set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    # 9) Attempt to solve
    solved = ss.solve(2)

    # If solved, optionally retrieve path
    if solved:
        print("Found solution path.")
        path = ss.getSolutionPath()
        path.printAsMatrix()

    del ss
    import gc
    gc.collect()

def test_control_rrt():
    # 1) Construct the SE2 state space
    space = ob.SE2StateSpace()
    
    # set R^2 bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # 2) Create a real-vector control space of dimension=2
    cspace = oc.RealVectorControlSpace(space, 2)
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-0.3)
    cbounds.setHigh(0.3)
    cspace.setBounds(cbounds)

    # 3) Construct a SpaceInformation from that (space, cspace)
    si = oc.SpaceInformation(space, cspace)
    si.setPropagationStepSize(1.0)

    # 4) Build a SimpleSetup from the SpaceInformation
    ss = oc.SimpleSetup(si)

    # 5) Provide a state validity checker as a lambda
    # This partial-lambda structure ensures the argument signature matches (State*) -> bool
    ss.setStateValidityChecker(lambda s: isStateValid(ss.getSpaceInformation(), s))
    
    # 6) Provide a state propagator
    ss.setStatePropagator(propagate)

    # 7) Create start and goal states
    start = space.allocState()
    start.setX(-0.5)
    start.setY(0.0)
    start.setYaw(0.0)

    goal = space.allocState()
    goal.setX(0.0)
    goal.setY(0.5)
    goal.setYaw(0.0)

    # 8) Set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    planner = oc.RRT(si)
    ss.setPlanner(planner)
    # 9) Attempt to solve
    solved = ss.solve(2)

    # If solved, optionally retrieve path
    if solved:
        print("Found solution path.")
        path = ss.getSolutionPath()
        path.printAsMatrix()

    del ss
    import gc
    gc.collect()