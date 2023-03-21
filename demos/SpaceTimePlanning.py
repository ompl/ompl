from os.path import abspath, dirname, join
import sys
import math
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from functools import partial

def isStateValid(spaceInformation, state):
    p = state[0][0]
    t = spaceInformation.getStateSpace().getStateTime(state)
    return t >= 0 and p < math.inf

class SpaceTimeMotionValidator(ob.MotionValidator):
    def checkMotion(self, s1, s2):
      if not si.isValid(s2):
        return False
      delta_pos = si.getStateSpace().distanceSpace(s1, s2)
      # delta_t = si.getStateSpace().distanceTime(s1, s2)
      t1 = si.getStateSpace().getStateTime(s1)
      t2 = si.getStateSpace().getStateTime(s2)
      delta_t = t2 - t1
      if delta_t <= 0 :
        return False
      if (delta_pos / delta_t) > si.getStateSpace().getVMax() :
        return False
      return True

vMax = 0.2
vector_space = ob.RealVectorStateSpace(1)
bounds = ob.RealVectorBounds(1)
bounds.setLow(-1.0)
bounds.setHigh(1.0)
vector_space.setBounds(bounds)

space = ob.SpaceTimeStateSpace(vector_space, vMax)
space.setTimeBounds(0.0, 10.0);

si = ob.SpaceInformation(space)
si.setMotionValidator(SpaceTimeMotionValidator(si))
si.setStateValidityChecker(ob.StateValidityCheckerFn( \
    partial(isStateValid, si)))

pdef = ob.ProblemDefinition(si)

start = ob.State(space)
start()[0][0] = 0.0
goal = ob.State(space)
goal()[0][0] = 1.0
pdef.setStartAndGoalStates(start, goal)

strrt = og.STRRTstar(si)
strrt.setRange(vMax)
strrt.setProblemDefinition(pdef)
strrt.setup()

result = strrt.solve(1.0)
print("Done planning.")

if result:
  print("Found path of length", pdef.getSolutionPath().length())
  print("Path:", pdef.getSolutionPath().printAsMatrix())
else:
  print("No solution found.")
