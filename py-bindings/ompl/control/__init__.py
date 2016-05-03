from ompl import base
from ompl.control._control import *

# call ompl.initializePlannerLists() to properly initialize this variable
# with a dictionary of dictionaries, containing planners and associated
# parameter info
planners = None

# type alias for std::function<void(const State*, const Control*, const double, State*)>
PostPropagationEvent = StatePropagatorFn
