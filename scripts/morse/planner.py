#!/usr/bin/env python3

import socket
import pickle
import sys

from ompl import control as oc

from environment import *

def planWithMorse(sockS, sockC):
    """
    Set up MyEnvironment, MorseSimpleSetup, and MyGoal objects.
    Plan using sockS as the socket to the Blender communicator script
    and sockC as the socket to the MORSE motion controller.
    """
    
    env = None
    try:
        # create a MORSE environment representation
        env = MyEnvironment(sockS, sockC)
        env.worldStepRes(0.1)   # stepsize
        
        # create a simple setup object
        ss = om.MorseSimpleSetup(env)
        si = ss.getSpaceInformation()
        
        # set up goal
        g = MyGoal(si, env)
        ss.setGoal(g)
        
        # choose a planner
        planner = oc.RRT(si)
        """
        planner = oc.KPIECE1(si)
        # requires a projection (there is a default, but it uses x,y positions of
        #  all rigid bodies; that could be a lot of dimensions)
        space = si.getStateSpace()
        proj = ExampleProjection(space)
        space.registerProjection("ExampleProjection", proj)
        planner.setProjectionEvaluator("ExampleProjection")
        """
        ss.setPlanner(planner)
        
        # solve
        ss.solve(120*60.0)
        
        # print the solution path
        if ss.haveSolutionPath():
            print("Saving solution.")
            cpath = ss.getSolutionPath()
            #cpath.interpolate()
            st = []
            con = []
            dur = []
            for i in range(cpath.getControlCount()):
                st.append(env.stateToList(cpath.getState(i)))
                con.append(tuple(cpath.getControl(i)[j] for j in range(env.cdesc[0])))
                dur.append(cpath.getControlDuration(i))
            st.append(env.stateToList(cpath.getState(cpath.getControlCount())))
            with open(sys.argv[1], 'wb') as f:
                pickle.dump((st,con,dur), f)
        else:
            print("No solution found.")
    
    except Exception as msg:
        if str(msg)!="[Errno 104] Connection reset by peer" \
            and str(msg)!="[Errno 32] Broken pipe": # ignore if user exits MORSE
            raise
    finally:
        # tell simulation it can shut down
        if env:
            env.endSimulation()

# set up the state and control sockets
sockS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sockC = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sockS.connect(('localhost', 50007))
sockC.connect(('localhost', 4000))

# plan
planWithMorse(sockS, sockC)



