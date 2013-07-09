#!/usr/bin/env python3

# Tests the OMPL MORSE extension and its Python bindings without invoking MORSE

from ompl import control as oc
from ompl import morse as om
from ompl import util as ou

def list2vec(l):
    """
    Convert a Python list into an ou.vectorDouble.
    """
    ret = ou.vectorDouble()
    for e in l:
        ret.append(e)
    return ret
    
class MyEnvironment(om.MorseEnvironment):
    """
    Represents the MORSE environment we will be planning in.
    Inherits from the C++ OMPL class om.MorseEnvironment and
    implements pure virtual functions readState(),
    writeState(), applyControl(), and worldStep().
    """
    
    def readState(self, state):
        """
        Get the state from the simulation and load it into
        the OMPL state.
        """
        # for each rigid body (4 components each)
        for i in range(0, self.rigidBodies_*4, 4):
            # set the pos, lin, ang components to (1,1,1)
            for j in range(3):
                state[i][j] = state[i+1][j] = state[i+2][j] = 1.0
            # set the quat component to the identity rotation
            state[i+3].w = 1.0
            state[i+3].x = state[i+3].y = state[i+3].z = 0.0
            
        
    def writeState(self, state):
        pass
        
    def applyControl(self, control):
        pass
        
    def worldStep(self, dur):
        pass

class MyProjection(om.MorseProjection):
    """
    The projection evaluator for the simulation.
    """
    
    def getDimension(self):
        return 2
    
    def project(self, state, projection):
        # use x and y coords of the cube
        projection[0] = state[0][0]
        projection[1] = state[0][1]
                
class MyGoal(om.MorseGoal):
    """
    The goal state of the simulation.
    """
    def __init__(self, si):
        super(MyGoal, self).__init__(si)
        self.c = 0
    
    def isSatisfied_Py(self, state):
        """
        Returns True on the 10th call.
        """
        self.c += 1
        self.distance_ = 10-self.c
        if self.distance_ == 0:
            return True
        return False

def planWithMorse():
    """
    Set up MyEnvironment and plan.
    """

    # create a MORSE environment representation
    env = MyEnvironment(2, 2, list2vec([-10,10,-1,1]), list2vec([-100,100,-100,100,-100,100]),
        list2vec([-10,10,-10,10,-10,10]), list2vec([-6,6,-6,6,-6,6]), 1.0/60, 30, 90)

    # create a simple setup object
    ss = om.MorseSimpleSetup(env)
    si = ss.getSpaceInformation()
    
    # set up the goal
    g = MyGoal(si)
    ss.setGoal(g)
    
    space = si.getStateSpace()
    proj = MyProjection(space)
    space.registerDefaultProjection(proj)
    
    # solve
    ss.solve(10)


# plan
planWithMorse()


