import sys
sys.path.insert(0, '/Users/weihang/Documents/Research/ompl/build/bindings')
from ompl import _base as ob

rvss = ob.RealVectorStateSpace(2)
print(rvss.getDimension())

rvBound = ob.RealVectorBounds(2)
rvBound.setLow(-1)
rvBound.setHigh(1)
rvss.setBounds(rvBound)

state = rvss.allocState()
sampler = ob.RealVectorStateSampler(rvss)
sampler.sampleUniform(state)
rvss.printState(state)
rvss.printSettings()