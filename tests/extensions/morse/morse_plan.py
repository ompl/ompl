#!/usr/bin/env python3

import sys
import socket
import time

#time.sleep(10)

from ompl import base as ob
from ompl import control as oc
from ompl import util as ou

def list2vec(l, ret=None):
    """
    Convert a Python list into an ou.vectorDouble.
      l = the list
      ret = an existing ou.vectorDouble or None
    Returns a new vector if ret=None; modifies ret in place otherwise.
    """
    if not ret:
        ret = ou.vectorDouble()
        for e in l:
            ret.append(e)
        return ret
    else:
        for i in range(len(l)):
            ret[i] = l[i]
    
class MyEnvironment(ob.MorseEnvironment):
    """
    Represents the MORSE environment we will be planning in.
    Inherits from the C++ OMPL class ob.MorseEnvironment and
    implements pure virtual functions prepareStateRead(),
    finalizeStateWrite(), applyControl(), and worldStep().
    """
    
    def prepareStateRead(self):
        """
        Get the state from the simulation and load it into
        the ou.vectorDoubles so OMPL can use it.
        """

        pos, lin, ang, quat = [], [], [], []
        for i in range(3*self.rigidBodies_):
            pos.append(1.0)
            lin.append(1.0)
            ang.append(1.0)
        for i in range(4*self.rigidBodies_):
            quat.append(1.0)
            if i%4:
                quat[i] = 0.0
        list2vec(pos, self.positions)
        list2vec(lin, self.linVelocities)
        list2vec(ang, self.angVelocities)
        list2vec(quat, self.quaternions)
        
    def finalizeStateWrite(self):
        """
        Compose a state string from the data in the
        ou.vectorDoubles and send it to the simulation.
        """
        pass
        
    def applyControl(self, control):
        """
        Tell MORSE to apply control to the robot.
        """
        pass
        
    def worldStep(self, dur):
        """
        Run the simulation for dur seconds. World tick is 1/60 s.
        """
        pass
        
    def endSimulation(self):
        """
        Let the simulation know to shut down.
        """
        pass
        
class MyGoal(ob.Goal):
    """
    The goal state of the simulation.
    """
    def __init__(self, si):
        super(MyGoal, self).__init__(si)
        self.c = 0
    
    def isSatisfied(self, state):
        self.c += 1
        if c==10:
            return True
        return false

def planWithMorse():
    """
    Set up MyEnvironment and plan.
    """
    
    try:
        # create a MORSE environment representation
        # TODO get these numbers from the simulation
        env = MyEnvironment(2, 2, list2vec([-10,10,-1,1]), list2vec([-100,100,-100,100,-100,100]),
            list2vec([-10,10,-10,10,-10,10]), list2vec([-6,6,-6,6,-6,6]))

        # create a simple setup object
        ss = oc.MorseSimpleSetup(env)
        
        # the right way to set up the goal, but oc::SpaceInformation isn't exposed, so we can't do this
        #g = MyGoal(ss.getSpaceInformation)
        
        # the wrong way; this will crash when the planner starts
        space = ss.getStateSpace()
        g = MyGoal(ob.SpaceInformation(space))
        
        ss.setGoal(g)
        
        # solve
        solved = ss.solve(1.0)
        print("Solve finished: %i", solved)
    
    finally:
        # tell simulation it can shut down
        env.endSimulation()
    
# plan
planWithMorse()


