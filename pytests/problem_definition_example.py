import sys
sys.path.insert(0, '/Users/weihang/Documents/Research/ompl/build/bindings')
from ompl import _base as ob

ss = ob.RealVectorStateSpace(2)
rvBound = ob.RealVectorBounds(2)
rvBound.setLow(-1)
rvBound.setHigh(1)
ss.setBounds(rvBound)
si = ob.SpaceInformation(ss)
pd = ob.ProblemDefinition(si)

start = ob.rv.ScopedState(ss)
start.random()
goal = ob.rv.ScopedState(ss)
goal.random()

print(f"Start State: {start}")
print(f"Goal State: {goal}")

pd.setStartAndGoalStates(start, goal)

print(f"Problem Definition: {pd}")