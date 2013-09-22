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

import math
import socket
import pickle

from ompl import base as ob
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

NO_ACK_MSG = 'ACK not received. Protocol violation!'

def unpickleFromSocket(s):
    """
    Retrieve and unpickle a pickle from a socket.
    """
    p = b''
    while True:
        try:
            p += s.recv(4096)   # keep adding more until we have it all
            o = pickle.loads(p)
        except EOFError:
            continue
        break
    return o

class MyEnvironment(om.MorseEnvironment):
    """
    Represents the MORSE environment we will be planning in.
    Inherits from the C++ OMPL class ob.MorseEnvironment.
    """
    
    def __init__(self, state_socket, control_socket, query_only=False):
        """
        Get information from Blender about the scene.
        """
        self.initState = None
        self.sockS = state_socket
        self.sockC = control_socket
        self.cdesc = self.call('getControlDescription()', b'')   # get info for applying controls
        if not self.cdesc:
            raise Exception("Can't communicate with MORSE process")
        if query_only:
            print('query done, ending simulation')
            self.call('endSimulation()')
            try:
                self.sockC.sendall(b"id simulation quit\n")
            except:
                pass
            return
        
        self.con = [0 for _ in range(self.cdesc[0])]    # cache of the last control set to MORSE
        cb = [self.cdesc[0]]            # control dimension
        cb.append(list2vec(self.cdesc[1]))   # control bounds
        
        rb = self.call('getRigidBodiesBounds()', b'')   # number of bodies and pos, lin, ang bounds
        if not rb:
            raise Exception("Can't communicate with MORSE process")
        for i in [1,2,3]:
            rb[i] = list2vec(rb[i])
        
        envArgs = cb + rb + [0.1, 10, 20]    # add step size, min/max control durations
        super(MyEnvironment, self).__init__(*envArgs)
        
        # tell MORSE to reset the simulation, because it was running while it was initializing
        try:
            self.sockC.sendall(b'id simulation reset_objects')
        except:
            raise Exception("Can't communicate with MORSE process")

    def call(self, cmd, pickdata=None):
        """
        Request a function call cmd from the simulation and
        return the result. If pickdata is b'', we're expecting
        pickled data to be returned over the socket. If pickdata
        has bytes, we'll send it over in a second transmission.
        """
        # submit cmd to socket; return unpickled response if present
        try:
            self.sockS.sendall(cmd.encode())
            if pickdata == b'':
                return unpickleFromSocket(self.sockS)
            elif pickdata:
                assert self.sockS.recv(1) == b'\x06', NO_ACK_MSG
                self.sockS.sendall(pickdata)
            assert self.sockS.recv(1) == b'\x06', NO_ACK_MSG
            return
        except:
            try:
                self.simRunning_ = False
            except:
                pass
            return
    
    def getGoalCriteria(self):
        """
        Get a list of tuples [(i_0,state_0),...] where i_n is
        the index of a rigid body in the world state and
        state_n is its goal position.
        """
        ret = self.call('getGoalCriteria()', b'')  # special receive pickled bytes
        if not ret:
            raise Exception("Can't communicate with MORSE process")
        return ret
    
    def readState(self, state):
        """
        Get the state from the simulation so OMPL can use it.
        """
        simState = self.call('extractState()', b'') # special receive pickled bytes
        if not simState:
            # we MUST continue somehow, so just return the initial state if possible
            simState = self.initState
            if not simState:
                # then no planning has actually happened yet, so just leave
                raise Exception("Can't communicate with MORSE process")
        
        if not self.initState:
            self.initState = simState[:]
        
        i = 0
        for obj in simState[:-1]:
            # for each rigid body
            for j in range(3):
                # copy a 3-vector (pos, lin, ang)
                for k in range(3):
                    state[i][k] = obj[j][k]
                i += 1
            # copy a 4-vector into the quaternion
            state[i].w = obj[3][0]
            state[i].x = obj[3][1]
            state[i].y = obj[3][2]
            state[i].z = obj[3][3]
            i += 1
        # copy the goalRegionSat flag
        state[i].value = simState[-1]
    
    def stateToList(self, state):
        simState = []
        for i in range(0, self.rigidBodies_*4, 4):
            # for each body
            simState.append((
                (state[i][0], state[i][1], state[i][2]),
                (state[i+1][0], state[i+1][1], state[i+1][2]),
                (state[i+2][0], state[i+2][1], state[i+2][2]),
                (state[i+3].w, state[i+3].x, state[i+3].y, state[i+3].z)
            ))
        # don't bother with the goalRegionSat flag
        return simState
    
    def writeState(self, state):
        """
        Compose a state string from the state data
        and send it to the simulation.
        """
        self.call('submitState()', pickle.dumps(self.stateToList(state)))  # special send pickled bytes
        
    def applyControl(self, control):
        """
        Tell MORSE to apply control to the robot.
        """
        con = [control[i] for i in range(len(control))] # make it iterable
        # If the control hasn't changed, we don't need to do anything
        if self.con != con:
            self.con = con
            i = 0
            for controller in self.cdesc[2:]:
                req = 'id %s %s %s\n' % (controller[0], controller[1], con[i:i+controller[2]])
                i += controller[2]
                try:
                    self.sockC.sendall(req.encode())
                except:
                    raise Exception("Can't communicate with MORSE process")
        
    def worldStep(self, dur):
        """
        Run the simulation for dur seconds.
        """
        for _ in range(round(dur/(1.0/60))):
            self.call('nextTick()')
        
    def endSimulation(self):
        """
        Let the simulation know to shut down.
        """
        if self.simRunning_:
            self.call('endSimulation()')
        try:
            self.sockC.sendall(b"id simulation quit\n")
        except:
            pass

class ExampleProjection(om.MorseProjection):
    """
    The projection for the simulation. Uses the x and y coordinates
    in the position component of the the first rigid body
    """
    
    def __init__(self, space):
        super(MyProjection, self).__init__(space)
        self.bounds_ = ob.RealVectorBounds(self.getDimension())
        self.subspaceIndex = 0  # first rigid body's position space
        for i in range(self.getDimension()):
            self.bounds_.low[i] = space.getSubspace(self.subspaceIndex).getBounds().low[i]
            self.bounds_.high[i] = space.getSubspace(self.subspaceIndex).getBounds().high[i]
        self.defaultCellSizes()
    
    def getDimension(self):
        return 2
    
    def defaultCellSizes(self):
        self.cellSizes_ = list2vec([1,1])
    
    def project(self, state, projection):
        # use x and y coords of the subspace
        projection[0] = state[self.subspaceIndex][0]
        projection[1] = state[self.subspaceIndex][1]

class MyGoal(om.MorseGoal):
    """
    The goal state of the simulation.
    """
    def __init__(self, si, env):
        """
        Initialize the goal and get list of criteria for satisfaction.
        """
        super(MyGoal, self).__init__(si)
        self.criteria = env.getGoalCriteria()
        self.rigidBodies_ = env.rigidBodies_
    
    def distLocRot(self, st, locrot):
        """
        How close are these rigid body states in position and orientation)?
        Use Euclidean distance for positions and calculate distance between
        orientations as 1-(<q1,q2>^2) where <q1,q2> is the inner product of the quaternions.
        """
        return (self.distLoc(st[0], locrot[0]), self.distRot(st[1], locrot[1]))
    
    def distLoc(self, st, loc):
        """
        How close are these rigid body states in position?
        """
        return math.sqrt(sum((st[i]-loc[i])**2 for i in range(3)))
    
    def distRot(self, st, rot):
        """
        How close are these rigid body states in orientation)?
        """
        # value in [0,1] where 0 means quats are the same
        return 1 - sum(st[i]*rot[i] for i in range(4))**2
    
    def isSatisfied_Py(self, state):
        """
        For every goal object, check if the rigid body object is close
        """
        self.distance_ = 0
        sat = True
        for crit in self.criteria:
            if len(crit) == 4:
                # this is a LocRot goal
                quat = state[4*crit[0]+3]
                stateTup = (state[4*crit[0]+0], (quat.w, quat.x, quat.y, quat.z))
                (dl,dr) = self.distLocRot(stateTup, crit[1])
                # Check tolerances for satisfaction
                if dl > crit[2] or dr > crit[3]:
                    sat = False
                self.distance_ += dl + dr
            elif len(crit) == 3:
                # this is a Rot goal
                quat = state[4*crit[0]+3]
                stateTup = (quat.w, quat.x, quat.y, quat.z)
                dr = self.distRot(stateTup, crit[1])
                # Check tolerance for satisfaction
                if dr > crit[2]:
                    sat = False
                self.distance_ += dr
            else:
                # this is a Region goal, don't alter sat here
                stateTup = state[4*crit[0]+0]
                self.distance_ += self.distLoc(stateTup, crit[1])
        
        # finally, check the goalRegionSat flag
        if not state[4*self.rigidBodies_].value:
            sat = False
        return sat

