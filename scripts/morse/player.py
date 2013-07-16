#!/usr/bin/env python3

import socket
import pickle

from environment import *

def playWithMorse(sockS, sockC):
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
        
        # play
        env.setMode('PLAY')
        f = open('path.out', 'rb')
        (st,con,dur) = pickle.load(f)
        for i in range(len(con)):
            # load state
            s = repr(st[i])
            s = s.replace('nan','float("nan")')
            s = s.replace('inf','float("inf")')
            env.call('submitState(%s)' % s)
            # apply control
            env.applyControl(con[i])
            # simulate
            env.worldStep(dur[i])
        # last state
        s = repr(st[len(con)])
        s = s.replace('nan','float("nan")')
        s = s.replace('inf','float("inf")')
        env.call('submitState(%s)' % s)
    
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
playWithMorse(sockS, sockC)
sockS.close()
sockC.close()


