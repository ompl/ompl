#!/usr/bin/env python3

import socket
import pickle
import sys
import time

from ompl.morse.environment import *

def playWithMorse(sockS, sockC):
    """
    Set up MyEnvironment object. Plan using sockS as the socket to the Blender
    communicator script and sockC as the socket to the MORSE motion controller.
    """
    
    env = None
    try:
        # create a MORSE environment representation
        env = MyEnvironment(sockS, sockC)
        env.worldStepRes(1.0/60)
        
        # play
        with open(sys.argv[1], 'rb') as f:
            (st,con,dur) = pickle.load(f)
        for i in range(len(con)):
            # load state
            env.call('submitState()', pickle.dumps(st[i]))
            # apply control
            print(con[i])
            env.applyControl(con[i])
            # simulate
            for _ in range(round(dur[i]/(1.0/60)/5)-10):
                env.worldStep(1.0/60)
                #time.sleep(1.0/60)  # to pace the playback
        # last state
        env.call('submitState()', pickle.dumps(st[len(con)]))
    
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

# play
playWithMorse(sockS, sockC)



