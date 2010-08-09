#!/bin/env python

from os.path import basename, abspath, dirname
import sys
try:
	from ompl import base as ob
	from ompl import geometric as og
except:
	# if the ompl module is not in the PYTHONPATH assume it is installed in the
	# parent directory
	sys.path.insert(0, dirname(dirname(abspath(__file__))))
	from ompl import base as ob
	from ompl import geometric as og
	
def isStateValid(spaceInformation, state):
	# Some arbitrary condition on the state (note that thanks to 
	# dynamic type checking we can just call getX() and do not need
	# to convert state to an SE2State.)
	return state.getX() < .6
	
def plan():
	# create an SE2 manifold
	manifold = ob.SE2StateManifold()
	
	# set lower and upper bounds
	bounds = ob.RealVectorBounds(2)
	bounds.setLow(-1)
	bounds.setHigh(1)
	manifold.setBounds(bounds)
	
	# create a simple setup object
	ss = og.SimpleSetup(manifold)
	ss.setStateValidityChecker(isStateValid)
	
	start = ob.SE2State(manifold)
	# we can pick a random start state...
	start.random()
	# ... or set specific values
	start().setX(.5)
	
	goal = ob.SE2State(manifold)
	# we can pick a random goal state...
	goal.random()
	# ... or set specific values
	goal().setY(-.5)
	
	ss.setStartAndGoalStates(ob.State(start), ob.State(goal))
	
	# this will automatically choose a default planner with 
	# default parameters
	solved = ss.solve(1.0)
	
	if solved:
		# try to shorten the path
		ss.simplifySolution()
		# print the simplified path
		print ss.getSolutionPath()


if __name__ == "__main__":
	plan()
