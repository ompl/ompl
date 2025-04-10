import sys
sys.path.insert(0, '/Users/weihang/Documents/Research/ompl/build/bindings')
from ompl import base as ob

rvss = ob.RealVectorStateSpace(2)
projectMatrix = ob.ProjectionMatrix()
projectMatrix.print()
print(1)
print(projectMatrix.mat)
print(2)
ob.RealVectorLinearProjectionEvaluator(rvss, projectMatrix.mat)