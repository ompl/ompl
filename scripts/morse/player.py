#!/usr/bin/env python3

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Caleb Voss

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



