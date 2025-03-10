import sys
sys.path.insert(0, '/Users/weihang/Documents/Research/ompl/build/bindings')
from ompl import _base as ob

rvss = ob.RealVectorStateSpace(2)
print(rvss.getDimension())

rvBound = ob.RealVectorBounds(2)
rvBound.setLow(-1)
rvBound.setHigh(1)
rvss.setBounds(rvBound)
rvss.printSettings()

state = rvss.allocState()
sampler = ob.RealVectorStateSampler(rvss)
sampler.sampleUniform(state)
rvss.printState(state)
state[0] = 0.5
state[1] = 0.5
rvss.printState(state)
state_another = rvss.allocState()
sampler.sampleUniform(state_another)
print(rvss.distance(state, state_another))
rvss.printState(state)
