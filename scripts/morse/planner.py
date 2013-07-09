#!/usr/bin/env python3

import socket
import time

from ompl import control as oc
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
    
class MyEnvironment(om.MorseEnvironment):
    """
    Represents the MORSE environment we will be planning in.
    Inherits from the C++ OMPL class ob.MorseEnvironment and
    implements pure virtual functions readState(),
    writeState(), applyControl(), and worldStep().
    """
    
    def __init__(self, state_socket, control_socket):
        """
        Get information from Blender about the scene.
        """
        self.sockS = state_socket
        self.sockC = control_socket
        self.simRunning = True
        self.con = (0,0)    # cache of the last control set to MORSE
        
        # tell MORSE to reset the simulation, because it was running while it was initializing
        self.sockC.sendall(b'id1 simulation reset_objects')
        
        cb = [2, list2vec([-10,10,-1,1])]   # TODO get from simulation
        
        rb = self.call('getRigidBodiesBounds()')    # number of bodies and positional bounds
        rb[1] = list2vec(rb[1])
        #TODO: edit C++ code to replace inf with std::numeric_limits<double>::max() etc.
        #inf = float('inf')
        #rb.append(list2vec([-inf, inf, -inf, inf, -inf, inf])) # lin bounds
        #rb.append(list2vec([-inf, inf, -inf, inf, -inf, inf])) # ang bounds
        rb.append(list2vec([-60, 60, -60, 60, -60, 60])) # lin bounds
        rb.append(list2vec([-60, 60, -60, 60, -60, 60])) # ang bounds
        
        super(MyEnvironment, self).__init__(cb[0], cb[1], rb[0], rb[1], rb[2], rb[3], 1.0/60, 30, 180)
        
    def call(self, cmd):
        """
        Request a function call cmd from the simulation and
        return the result.
        """
        # submit cmd to socket; return eval()'ed response
        try:
            self.sockS.sendall(cmd.encode())
            return eval(sockS.recv(16384))    # TODO: buffer size? states can get pretty big
        except:
            self.simRunning = False
            raise
    
    def getGoalCriteria(self):
        """
        Get a list of tuples [(i_0,state_0),...] where i_n is
        the index of a rigid body in the world state and
        state_n is its goal position.
        """
        return self.call('getGoalCriteria()')
    
    def readState(self, state):
        """
        Get the state from the simulation so OMPL can use it.
        """
        simState = self.call('extractState()')
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
        
    def writeState(self, state):
        """
        Compose a state string from the state data
        and send it to the simulation.
        """
        simState = []
        for i in range(0, self.rigidBodies_*4, 4):
            # for each body
            simState.append((
                (state[i][0], state[i][1], state[i][2]),
                (state[i+1][0], state[i+1][1], state[i+1][2]),
                (state[i+2][0], state[i+2][1], state[i+2][2]),
                (state[i+3].w, state[i+3].x, state[i+3].y, state[i+3].z)
            ))
        
        # make safe for eval()
        s = repr(simState)
        s = s.replace('nan','float("nan")')
        s = s.replace('inf','float("inf")')
        
        self.call('submitState(%s)' % s)
        
    def applyControl(self, control):
        """
        Tell MORSE to apply control to the robot.
        """
        if self.con != (control[0],control[1]):
            self.con = (control[0],control[1])
            sockC.sendall(('id1 robot.motion set_speed [%f,%f]\n' % self.con).encode())
        
    def worldStep(self, dur):
        """
        Run the simulation for dur seconds.
        """
        for i in range(int(round(dur/(1.0/60)))):
            self.call('nextTick()')
    
    def setSpeed(self, speed):
        """
        Set the time multiplier for the simulation speed.
        """
        self.call('setSpeed(%f)' % speed)
        
    def endSimulation(self):
        """
        Let the simulation know to shut down.
        """
        if self.simRunning:
            self.call('endSimulation()')

class MyProjection(om.MorseProjection):
    """
    The projection evaluator for the simulation.
    """
    
    def getDimension(self):
        return 2
    
    def defaultCellSizes(self):
        # grid for robot x,y location
        self.cellSizes_ = list2vec([2,2])
    
    def project(self, state, projection):
        # use x and y coords of the robot
        projection[0] = state[1*4+0][0]
        projection[1] = state[1*4+0][1]

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
    
    def dist(self, s1, s2):
        """
        How close are these rigid body states (position and orientation)?
        Calculates Euclidean distance between positions and distance between
        orientations as 1-(<q1,q2>^2) where <q1,q2> is the inner product of the quaternions.
        """
        return sum((s1[0][i]-s2[0][i])**2 for i in range(3)) + \
            (1 - sum(s1[3][i]*s2[3][i] for i in range(4))**2)   # [0,1], where 0 means quats are the same
    
    def isSatisfied_Py(self, state):
        """
        For every goal object, check if the rigid body object is close
        """
        self.distance = 0
        for crit in self.criteria:
            quat = state[4*crit[0]+3]
            stateTup = (state[4*crit[0]+0], state[4*crit[0]+1],
                        state[4*crit[0]+2], (quat.w, quat.x, quat.y, quat.z))
            self.distance += self.dist(stateTup, crit[1])
        if self.distance > len(crit)*0.1:
            return False
        return True


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
        ss.setPlanner(planner)
        
        # use a specific projection
        space = si.getStateSpace()
        proj = MyProjection(space)
        space.registerDefaultProjection(proj)
        
        # solve
        env.setSpeed(16)
        ss.solve(10*60.0)   # 10 minutes
        env.setSpeed(1)
        
        # print the solution path
        input("Press enter to see solution...")
        ss.playSolutionPath()
    
    except OSError as msg:
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


