#!/bin/env python

from math import sin, cos
from os.path import basename, abspath, dirname
import sys
try:
	from ompl import base as ob
	from ompl import control as oc
except:
	# if the ompl module is not in the PYTHONPATH assume it is installed in the
	# parent directory
	sys.path.insert(0, dirname(dirname(abspath(__file__))))
	from ompl import base as ob
	from ompl import control as oc

def isStateValid(spaceInformation, state):
	# perform collision checking or check if other constraints are
	# satisfied
	return True #spaceInformation.satiesfiesBounds(state)

def propagate(start, control, duration, state):
	state.setX( start.getX() + control[0] * cos(start.getYaw()) )
	state.setY( start.getY() + control[0] * sin(start.getYaw()) )
	state.setYaw( start.getYaw() + control[1] )
	return oc.PROPAGATION_START_UNKNOWN
	
def plan():
	manifold = ob.SE2StateManifold()
	
	bounds = ob.RealVectorBounds(2)
	bounds.setLow(-1)
	bounds.setHigh(1)
	
	manifold.setBounds(bounds)
	
	cmanifold = oc.RealVectorControlManifold(manifold, 2)
	
	cbounds = oc.RealVectorBounds(2)
	cbounds.setLow(-.3)
	cbounds.setHigh(.3)
	
	cmanifold.setBounds(cbounds)
	
	cmanifold.setPropagationFunction(propagate)
	
	ss = oc.SimpleSetup(cmanifold)
	ss.setStateValidityChecker(isStateValid)
	
	start = ob.State(manifold)
	start().setX(-0.5);
	start().setY(0.0);
	start().setYaw(0.0);
	
	goal = ob.State(manifold);
	goal().setX(0.0);
	goal().setY(0.5);
	goal().setYaw(0.0);
	
	ss.setStartAndGoalStates(start, goal, 0.05)
	
	solved = ss.solve(10.0)
	
	if solved:
		print "Found solution:", ss.getSolutionPath().asGeometric()
	
if __name__ == "__main__":
	plan()
