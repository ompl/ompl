from ompl import base
from ompl.geometric._geometric import *

# single-threaded version of PRM*
class PRMstar(PRM):
    def __init__(self, si):
        super(PRMstar, self).__init__(si, True)
        self.setName("PRMstar")
    def setMaxNearestNeighbors(self, k):
        # make this a no-op to preserve k* connection strategy
        return

class LazyPRMstar(LazyPRM):
    def __init__(self, si):
        super(LazyPRMstar, self).__init__(si, True)
        self.setName("LazyPRMstar")
    def setMaxNearestNeighbors(self, k):
        # make this a no-op to preserve k* connection strategy
        return

# call ompl.initializePlannerLists() to properly initialize this variable
# with a dictionary of dictionaries, containing planners and associated
# parameter info
planners = None
