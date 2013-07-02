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
    
    def setSockets(self, state_socket, control_socket):
        """
        Use comm_socket for communication with the simulation.
        """
        self.sockS = state_socket
        self.sockC = control_socket
        self.simRunning = True
        self.con = (0,0)    # cache of the last control set to MORSE
        # tell MORSE to reset the simulation, because it was running while it was initializing
        self.sockC.sendall(b'id1 simulation reset_objects')
        
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
            #print("Quat: %f,%f,%f,%f" % (state[i].w, state[i].x, state[i].y, state[i].z))
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
        self.call('submitState(%s)' % repr(simState))
        
    def applyControl(self, control):
        """
        Tell MORSE to apply control to the robot.
        """
        if self.con != (control[0],control[1]):
            self.con = (control[0],control[1])
            sockC.sendall(('id1 robot.motion set_speed [%f,%f]\n' % self.con).encode())
        
    def worldStep(self, dur):
        """
        Run the simulation for dur seconds. World tick is 1/60 s.
        """
        for i in range(int(round(dur/(1.0/60)))):
            self.call('nextTick()')
        
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
        # coarse grid for cube x,y location
        # fine grid for car x,y location
        cellSizes_ = list2vec([2.0,2.0])
    
    def project(self, state, projection):
        # use x and y coords of the robot
        projection[0] = state[0*4][0]
        projection[1] = state[0*4][1]

class MyGoal(om.MorseGoal):
    """
    The goal state of the simulation.
    """
    
    def isSatisfied_Py(self, state):
        # come to rest on the platform within 3 taxicab units of (0,12)
        # body list: 0:__robot, 1,2,3,4:_wheel*
        linvel = abs(state[0*4+1][0])+abs(state[0*4+1][1])+abs(state[0*4+1][2])
        self.distance_ = abs(state[0*4][0] - 0) + \
            abs(state[0*4][1] - -12) + linvel
        return self.distance_ < 3.0


def planWithMorse(sockS, sockC):
    """
    Set up MyEnvironment, MorseStateSpace, and MorseSimpleSetup objects.
    Plan using sockS as the socket to the Blender communicator script
    and sockC as the socket to the MORSE motion controller.
    """
    
    try:
        # create a MORSE environment representation
        # TODO get these numbers from the simulation
        env = MyEnvironment(5, 2, list2vec([-10,10,-1,1]), list2vec([-3,3,-15,7,-1,5]),
            list2vec([-20,20,-20,20,-20,20]), list2vec([-60,60,-60,60,-60,60]), 60, 120)
        env.setSockets(sockS, sockC)
        
        # create a simple setup object
        ss = om.MorseSimpleSetup(env)
        si = ss.getSpaceInformation()
        
        # set up goal
        g = MyGoal(si)
        ss.setGoal(g)
        
        # choose a planner
        planner = oc.RRT(si)
        ss.setPlanner(planner)
        
        # use a specific projection
        #space = si.getStateSpace()
        #proj = MyProjection(space)
        #space.registerDefaultProjection(proj)
        
        # solve
        ss.solve(20*60.0)
        
        # print the solution path
        input("Press enter to see solution...")
        ss.playSolutionPath()
    
    except:
        pass
    finally:
        # tell simulation it can shut down
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


