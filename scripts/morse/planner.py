#!/usr/bin/env python3

import socket
import pickle

from ompl import control as oc

from environment import *

def planWithMorse(sockS, sockC):
    """
    Set up MyEnvironment, MorseSimpleSetup, and MyGoal objects.
    Plan using sockS as the socket to the Blender communicator script
    and sockC as the socket to the MORSE motion controller.
    """
    
    try:
        # create a MORSE environment representation
        env = MyEnvironment(sockS, sockC)
        
        # create a simple setup object
        ss = om.MorseSimpleSetup(env)
        si = ss.getSpaceInformation()
        
        # set up goal
        g = MyGoal(si, env)
        ss.setGoal(g)
        
        # choose a planner
        planner = oc.KPIECE1(si)
        # with a specific projection
        space = si.getStateSpace()
        proj = MyProjection(space)
        space.registerProjection("MyProjection", proj)
        planner.setProjectionEvaluator("MyProjection")
        
        ss.setPlanner(planner)
        
        # solve
        env.setMode('PLAN')
        ss.solve(5*60.0)
        
        # print the solution path
        if ss.haveSolutionPath():
            print("Saving solution.")
            cpath = ss.getSolutionPath()
            st = []
            con = []
            dur = []
            for i in range(cpath.getControlCount()):
                st.append(env.stateToList(cpath.getState(i)))
                con.append((cpath.getControl(i)[0], cpath.getControl(i)[1]))
                dur.append(cpath.getControlDuration(i))
            st.append(env.stateToList(cpath.getState(cpath.getControlCount())))
            f = open('path.out', 'wb')
            pickle.dump((st,con,dur), f)
            f.close()
        else:
            print("No solution found.")
    
    except Exception as msg:
        if str(msg)!="[Errno 104] Connection reset by peer": # this happens when MORSE exits
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
sockS.close()
sockC.close()


