import sys
sys.path.insert(0, '/Users/weihang/Documents/Research/ompl/build/bindings')
from ompl import base as ob

status = ob.PlannerStatus(ob.PlannerStatusType.INVALID_START)
print(status)
print(int(status))