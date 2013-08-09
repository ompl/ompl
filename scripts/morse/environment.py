
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
    Inherits from the C++ OMPL class ob.MorseEnvironment and
    implements pure virtual functions readState(),
    writeState(), applyControl(), and worldStep().
    """
    
    def __init__(self, state_socket, control_socket, query_only=False):
        """
        Get information from Blender about the scene.
        """
        self.simRunning = True
        self.sockS = state_socket
        self.sockC = control_socket
        self.cdesc = self.call('getControlDescription()', b'')   # get info for applying controls
        if query_only:
            self.endSimulation()
            return
        
        self.con = [0 for _ in range(self.cdesc[0])]    # cache of the last control set to MORSE
        cb = [self.cdesc[0]]            # control dimension
        cb.append(list2vec(self.cdesc[1]))   # control bounds
        
        rb = self.call('getRigidBodiesBounds()', b'')   # number of bodies and pos, lin, ang bounds
        for i in [1,2,3]:
            rb[i] = list2vec(rb[i])
        
        envArgs = cb + rb + [0.1, 5, 10]    # add step size, min/max control durations
        super(MyEnvironment, self).__init__(*envArgs)
        
        # tell MORSE to reset the simulation, because it was running while it was initializing
        self.sockC.sendall(b'id simulation reset_objects')

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
            self.simRunning = False
            raise
    
    def getGoalCriteria(self):
        """
        Get a list of tuples [(i_0,state_0),...] where i_n is
        the index of a rigid body in the world state and
        state_n is its goal position.
        """
        return self.call('getGoalCriteria()', b'')  # special receive pickled bytes
    
    def readState(self, state):
        """
        Get the state from the simulation so OMPL can use it.
        """
        simState = self.call('extractState()', b'') # special receive pickled bytes
        
        i = 0
        for obj in simState:
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
                self.sockC.sendall(req.encode())
    
    def worldStepRes(self, dur):
        """
        Configure simulation to run in dur second intervals.
        """
        self.call('stepRes(%f)'%dur)
        
    def worldStep(self, dur):
        """
        Run the simulation for worldStepRes seconds (not dur!).
        """
        #for _ in range(round(dur/(0.1))):
        self.call('nextTick()')
        
    def endSimulation(self):
        """
        Let the simulation know to shut down.
        """
        if self.simRunning:
            self.call('endSimulation()')
        self.sockC.sendall(b"id simulation quit\n")

class MyProjection(om.MorseProjection):
    """
    The projection evaluator for the simulation. Uses the x and y coordinates
    in the position component of the robot.
    """
    
    def __init__(self, space):
        super(MyProjection, self).__init__(space)
        self.bounds_ = ob.RealVectorBounds(self.getDimension())
        self.robotPosSpaceIndex = 4 # TODO: figure out automatically which components are the robot positions
        for i in range(self.getDimension()):
            self.bounds_.low[i] = space.getSubspace(self.robotPosSpaceIndex).getBounds().low[i]
            self.bounds_.high[i] = space.getSubspace(self.robotPosSpaceIndex).getBounds().high[i]
        self.defaultCellSizes()
    
    def getDimension(self):
        return 2
    
    def defaultCellSizes(self):
        # grid for robot x,y locations
        self.cellSizes_ = list2vec([2,2])
    
    def project(self, state, projection):
        # use x and y coords of the robots
        projection[0] = state[self.robotPosSpaceIndex][0]
        projection[1] = state[self.robotPosSpaceIndex][1]

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
    
    def distLocRot(self, st, locrot):
        """
        How close are these rigid body states in position and orientation)?
        Use Euclidean distance for positions and calculate distance between
        orientations as 1-(<q1,q2>^2) where <q1,q2> is the inner product of the quaternions.
        """
        return (math.sqrt(sum((st[0][i]-locrot[0][i])**2 for i in range(3))),
            self.distRot(st[1], locrot[1]))
    
    def distRot(self, st, rot):
        """
        How close are these rigid body states in orientation)?
        Calculate distance between orientations as 1-(<q1,q2>^2) where <q1,q2>
        is the inner product of the quaternions.
        """
        # value in [0,1] where 0 means quats are the same
        return 1 - sum(st[i]*rot[i] for i in range(4))**2
    
    def isSatisfied_Py(self, state):
        """
        For every goal object, check if the rigid body object is close
        """
        self.distance = 0
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
                self.distance += dl + dr
            elif len(crit) == 3:
                # this is a Rot goal
                quat = state[4*crit[0]+3]
                stateTup = (quat.w, quat.x, quat.y, quat.z)
                dr = self.distRot(stateTup, crit[1])
                # Check tolerance for satisfaction
                if dr > crit[2]:
                    sat = False
                self.distance += dr
            else:
                # this is a Region goal
                pass
        print(self.distance)
        return sat

