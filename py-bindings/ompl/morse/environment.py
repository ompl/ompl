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

import math
import pickle

from ompl import base as ob
from ompl import morse as om
from ompl import util as ou

##
# \brief Convert a Python list into an ou.vectorDouble
def list2vec(l):

    ret = ou.vectorDouble()
    for e in l:
        ret.append(e)
    return ret

##
# \brief Retrieve and unpickle a pickle from a socket
def unpickleFromSocket(s):

    p = b''
    while True:
        try:
            # keep adding more pickle data until we have it all
            p += s.recv(4096)
            o = pickle.loads(p)
        except EOFError:
            # pickle.loads failed because the pickle wasn't complete
            continue
        break
    return o

## \brief Controls are applied in discrete time intervals of this size (in seconds)
controlStepSize = 0.1
## \brief Smallest number of times a given control may be applied consecutively
minControlDuration = 10
## \brief Largest number of times a given control may be applied consecutively
maxControlDuration = 20

##
# \brief Represents the MORSE environment we will be planning in; inherits from the C++ class
class MyEnvironment(om.MorseEnvironment):

    ##
    # \brief Constructor, get information from Blender about the scene;
    #     accepts the sockets for transferring state data and control data,
    #     as well as a flag indicating whether we are only interested in
    #     retrieving a description of the control setup for the simulation
    def __init__(self, state_socket, control_socket, query_only=False):

        ## \brief A saved copy of the initial state of the system
        self.initState = None

        ## \brief Sockets used to talk to communicator.py and MORSE respectively
        self.sockS = state_socket
        self.sockC = control_socket

        ## \brief Description of available control inputs for the system
        self.cdesc = self.call('getControlDescription()', b'')
        if not self.cdesc:
            raise Exception("Can't communicate with MORSE process")

        # If we're only here to get the cdesc, then we're done
        if query_only:
            print('query done, ending simulation')
            self.call('endSimulation()')
            try:
                self.sockC.sendall(b"id simulation quit\n")
            except:
                pass
            return

        ## \brief cache of the last control sent to MORSE
        self.con = [0 for _ in range(self.cdesc[0])]

        # control dimension and control bounds
        cb = [self.cdesc[0]]
        cb.append(list2vec(self.cdesc[1]))

        # number of bodies and positional, linear, and angular velocity bounds
        rb = self.call('getRigidBodiesBounds()', b'')
        if not rb:
            raise Exception("Can't communicate with MORSE process")
        for i in [1, 2, 3]:
            # Make them suitable for sending to the C++ constructor
            rb[i] = list2vec(rb[i])

        # Combine control and rigid body info with step size and min/max control durations
        envArgs = cb + rb + [controlStepSize, minControlDuration, maxControlDuration]
        super(MyEnvironment, self).__init__(*envArgs)

        # Tell MORSE to reset the simulation, because it was running during initializing
        try:
            self.sockC.sendall(b'id simulation reset_objects')
        except:
            raise Exception("Can't communicate with MORSE process")

    ##
    # \brief Request a function call from the simulation and
    #    return the result. If pickdata is b'', we're expecting
    #    pickled data to be returned over the socket. If pickdata
    #    has bytes, we'll send them over in a second transmission
    def call(self, cmd, pickdata=None):

        # Submit cmd to socket; return unpickled response if present
        try:
            self.sockS.sendall(cmd.encode())
            if pickdata == b'':
                # Expecting response data
                return unpickleFromSocket(self.sockS)
            elif pickdata:
                assert self.sockS.recv(1) == b'\x06', 'ACK not received. Protocol violation!'
                # He'll be expecting this extra data
                self.sockS.sendall(pickdata)
            assert self.sockS.recv(1) == b'\x06', 'ACK not received. Protocol violation!'
            return
        except:
            # If something broke, we'll start the clean up process
            self.simRunning_ = False
            return

    ##
    # \brief Get a list of tuples containing the state index of
    #    a rigid body and its desired goal position for each
    #    pose or rotation goal
    def getGoalCriteria(self):

        # The response to this call should be what we want
        ret = self.call('getGoalCriteria()', b'')
        if not ret:
            raise Exception("Can't communicate with MORSE process")
        return ret

    ##
    # \brief Get the state from the simulation so OMPL can use it
    def readState(self, state):

        simState = self.call('extractState()', b'')
        if not simState:
            # We MUST continue somehow, just until the PlannerTerminationCondition
            #   is evaluated again; return the initial state if possible, so as not
            #   to corrupt any progress up to this point
            simState = self.initState
            if not simState:
                # then no planning has actually happened yet, so just leave with an error
                raise Exception("Can't communicate with MORSE process")

        # Initialize the copy of the initial state if this is the first time through
        if not self.initState:
            self.initState = simState[:]

        # For each rigid body
        i = 0   # subspace counter
        for obj in simState[:-1]:
            # Copy the three 3-vectors: pos, lin, and ang
            for j in range(3):
                for k in range(3):
                    state[i][k] = obj[j][k]
                i += 1
            # Copy the 4-vector into the quaternion subspace
            state[i].w = obj[3][0]
            state[i].x = obj[3][1]
            state[i].y = obj[3][2]
            state[i].z = obj[3][3]
            i += 1
        # Copy the goalRegionSat flag
        state[i].value = simState[-1]

    ##
    # \brief Convert the OMPL state into a Python list of tuples
    def stateToList(self, state):

        simState = []
        # For each body, compose the state tuple
        for i in range(0, self.rigidBodies_*4, 4):
            simState.append((
                (state[i][0], state[i][1], state[i][2]),
                (state[i+1][0], state[i+1][1], state[i+1][2]),
                (state[i+2][0], state[i+2][1], state[i+2][2]),
                (state[i+3].w, state[i+3].x, state[i+3].y, state[i+3].z)
            ))
        # don't care about the goalRegionSat flag
        return simState

    ##
    # \brief Send OMPL's state to the simulation by pickling the list-of-tuples representation
    def writeState(self, state):

        self.call('submitState()', pickle.dumps(self.stateToList(state)))

    ##
    # \brief Tell MORSE what controls to apply to the robot
    def applyControl(self, control):

        # Create a sliceable version of the data
        con = [control[i] for i in range(len(control))]

        # Only do something if the control changed since last time
        if self.con != con:
            self.con = con
            # For each exposed controller service
            i = 0
            for controller in self.cdesc[2:]:
                # Compose the request by grabbing the appropriate number of control values
                req = 'id %s %s %s\n' % (controller[0], controller[1], con[i:i+controller[2]])
                i += controller[2]
                try:
                    self.sockC.sendall(req.encode())
                except:
                    raise Exception("Can't communicate with MORSE process")

    ##
    # \brief Run the simulation for the given number of seconds
    def worldStep(self, dur):

        # Ticks are at 60 Hz
        for _ in range(round(dur/(1.0/60))):
            self.call('nextTick()')

    ##
    # \brief Let the simulation know to shut down
    def endSimulation(self):

        if self.simRunning_:
            # Inform the logic script (communicator.py)
            self.call('endSimulation()')
        try:
            # Inform MORSE
            self.sockC.sendall(b"id simulation quit\n")
        except:
            pass


##
# \brief An example projection for the simulation. Uses the x and y coordinates
#    in the position component of the the first rigid body
class ExampleProjection(om.MorseProjection):

    ##
    # \brief Constructor to setup projection specfications;
    #    accepts the ompl.control.SpaceInformation object
    def __init__(self, space):

        super(ExampleProjection, self).__init__(space)

        ## \brief Convenience parameter for defining which subspace our
        #     projection uses; in this case, the position space of the first object
        self.subspaceIndex = 4*0 + 0

        ## \brief Bounds for the projection space (used by OMPL)
        self.bounds_ = ob.RealVectorBounds(self.getDimension())
        for i in range(self.getDimension()):
            self.bounds_.low[i] = space.getSubspace(self.subspaceIndex).getBounds().low[i]
            self.bounds_.high[i] = space.getSubspace(self.subspaceIndex).getBounds().high[i]

        self.defaultCellSizes()

    ##
    # \brief Dimension of the projection space
    def getDimension(self):
        # We're only interested in x,y position
        return 2

    ##
    # \brief Set up the default size of the cells in the projection
    def defaultCellSizes(self):

        # Cells will measure 1 Blender unit by 1 Blender unit
        self.cellSizes_ = list2vec([1, 1])

    ##
    # \brief Get the projected coordinates of a state
    def project(self, state, projection):

        # Use x and y coords of the specified subspace
        projection[0] = state[self.subspaceIndex][0]
        projection[1] = state[self.subspaceIndex][1]


##
# \brief Representation of the goal state of the simulation
class MyGoal(om.MorseGoal):

    ##
    # \brief Constructor accepting ompl.control.SpaceInformation and
    #    MyEnvironment objects
    def __init__(self, si, env):

        super(MyGoal, self).__init__(si)

        ## \brief The list of criteria for this goal to be satisfied
        self.criteria = env.getGoalCriteria()

        ## \brief Subspace index where the goalRegionSat flag is kept
        self.satFlagIndex = 4*env.rigidBodies_

    ##
    # \brief How close are these rigid body states in position and orientation?
    def distLocRot(self, st, locrot):

        # Return tuple containing positial and rotation distances
        return (self.distLoc(st[0], locrot[0]), self.distRot(st[1], locrot[1]))

    ##
    # \brief How close are these rigid body states in position?
    def distLoc(self, st, loc):
        # Compoute Euclidean distance
        return math.sqrt(sum((st[i]-loc[i])**2 for i in range(3)))

    ##
    # \brief How close are these rigid body states in orientation?
    def distRot(self, st, rot):
        # 1-(<q1,q2>^2) where <q1,q2> is the inner product of the quaternions;
        #   yields a value in [0,1] where 0 means quats are the same
        return 1 - sum(st[i]*rot[i] for i in range(4))**2

    ##
    # \brief Check whether we satisfy the goal criteria and set the distance-to-goal
    def isSatisfied_Py(self, state):

        # This distance-to-goal variable is read by the C++ code after this function call
        self.distance_ = 0

        sat = True
        # For every goal criterion
        for crit in self.criteria:
            if len(crit) == 4:

                # This is a LocRot goal; get positional and rotational distance
                quat = state[4*crit[0]+3]
                stateTup = (state[4*crit[0]+0], (quat.w, quat.x, quat.y, quat.z))
                (dl, dr) = self.distLocRot(stateTup, crit[1])

                # Check tolerances for satisfaction
                if dl > crit[2] or dr > crit[3]:
                    sat = False

                self.distance_ += dl + dr

            elif len(crit) == 3:

                # This is a Rot goal; get rotational distance
                quat = state[4*crit[0]+3]
                stateTup = (quat.w, quat.x, quat.y, quat.z)
                dr = self.distRot(stateTup, crit[1])

                # Check tolerance for satisfaction
                if dr > crit[2]:
                    sat = False

                self.distance_ += dr
            else:

                # This is a Region goal; get positional distance;
                #   (satisfaction computed elsewhere)
                stateTup = state[4*crit[0]+0]
                self.distance_ += self.distLoc(stateTup, crit[1])

        # Finally, check the goalRegionSat flag
        if not state[self.satFlagIndex].value:
            sat = False

        # Whether any criterion was not satisfied
        return sat
