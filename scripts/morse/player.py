#!/usr/bin/env python3

import socket
import pickle
import sys

from environment import *

def playWithMorse(sockS, sockC):
    """
    Set up MyEnvironment object. Plan using sockS as the socket to the Blender
    communicator script and sockC as the socket to the MORSE motion controller.
    """
    
    try:
        # create a MORSE environment representation
        env = MyEnvironment(sockS, sockC)
        
        # play
        f = open(sys.argv[1], 'rb')
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
        if str(msg)!="[Errno 104] Connection reset by peer": # ignore if user exits MORSE
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

# play
playWithMorse(sockS, sockC)



