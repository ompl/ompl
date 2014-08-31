from ompl import base
from ompl.geometric._geometric import *

# single-threaded version of PRM*
class PRMstar(PRM):
    def __init__(self, si):
        super(PRMstar, self).__init__(si, True)
        self.setName("PRMstar")

# call ompl.initializePlannerLists() to properly initialize this variable
# with a dictionary of dictionaries, containing planners and associated
# parameter info
planners = None
