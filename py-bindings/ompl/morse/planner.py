#!/usr/bin/env python3

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2013, Rice University
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

from ompl import control as oc
from ompl.morse.environment import *

##
# \brief Set up MyEnvironment, MorseSimpleSetup, and MyGoal objects.
#    Plan using sockS as the socket to the Blender communicator script
#    and sockC as the socket to the MORSE motion controller.
def planWithMorse(sockS, sockC):

    env = None
    try:
        # Create a MORSE environment representation
        env = MyEnvironment(sockS, sockC)

        # Create a simple setup object
        ss = om.MorseSimpleSetup(env)
        si = ss.getSpaceInformation()

        # Set up goal
        g = MyGoal(si, env)
        ss.setGoal(g)

        # Choose a planner
        planner = oc.RRT(si)
        """
        # Alternative setup with a planner using a projection
        planner = oc.KPIECE1(si)
        space = si.getStateSpace()
        # This projection uses the x,y coords of every rigid body in the state space.
        proj = om.MorseProjection(space)
        space.registerProjection("MorseProjection", proj)
        planner.setProjectionEvaluator("MorseProjection")
        """

        ss.setPlanner(planner)

        # Solve
        ss.solve()

        # Write the solution path to file
        if ss.haveSolutionPath():
            solnFileName = sys.argv[sys.argv.index('--') + 1]
            print("Saving solution to '" + solnFileName + "'...")
            cpath = ss.getSolutionPath()
            # Save the states, controls, and durations
            st = []
            con = []
            dur = []
            for i in range(cpath.getControlCount()):
                st.append(env.stateToList(cpath.getState(i)))
                con.append(tuple(cpath.getControl(i)[j] for j in range(env.cdesc[0])))
                dur.append(cpath.getControlDuration(i))
            st.append(env.stateToList(cpath.getState(cpath.getControlCount())))
            with open(solnFileName, 'wb') as f:
                # Pickle it all into a file
                pickle.dump((st, con, dur), f)
            print("...done.")
        else:
            print("No solution found.")

    except Exception as msg:
        # Ignore errors caused by MORSE or Blender shutting down
        if str(msg) != "[Errno 104] Connection reset by peer" \
          and str(msg) != "[Errno 32] Broken pipe":
            raise

    finally:
        # Tell simulation it can shut down
        if env:
            env.endSimulation()

# Set up the state and control sockets
sockS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sockC = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sockS.connect(('localhost', 50007))
sockC.connect(('localhost', 4000))

# Plan
planWithMorse(sockS, sockC)

# Quit this instance of Blender.
exit(0)
