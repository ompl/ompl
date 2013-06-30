#!/usr/bin/env python3

import socket

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
    
    def setSocket(self, comm_socket):
        """
        Use comm_socket for communication with the simulation.
        """
        self.sock = comm_socket
        
    def call(self, cmd):
        """
        Request a function call cmd from the simulation and
        return the result.
        """

        # submit cmd to socket; return eval()'ed response
        if sock:
            self.sock.sendall(cmd.encode())
            return eval(sock.recv(1024))    # TODO: buffer size? states can get pretty big
    
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
        # TODO
        pass
        
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
        self.call('endSimulation()')
        
class MyGoal(om.MorseGoal):
    """
    The goal state of the simulation.
    """
    def __init__(self, si):
        super(MyGoal, self).__init__(si)
        self.c = 0
    
    def isSatisfied_Py(self, state):
        # TODO
        self.c += 1
        if self.c==10:
            return True
        return False

def planWithMorse(sock):
    """
    Set up MyEnvironment, MorseStateSpace, and MorseSimpleSetup objects.
    Plan using sock as the communication socket to the simulation.
    """
    
    try:
        # create a MORSE environment representation
        # TODO get these numbers from the simulation
        env = MyEnvironment(2, 2, list2vec([-10,10,-1,1]), list2vec([-100,100,-100,100,-100,100]),
            list2vec([-10,10,-10,10,-10,10]), list2vec([-6,6,-6,6,-6,6]))
        env.setSocket(sock)

        # create a simple setup object
        ss = om.MorseSimpleSetup(env)
        
        # set up goal
        g = MyGoal(ss.getSpaceInformation())
        ss.setGoal(g)
        
        # solve
        ss.solve(20.0)
    
    finally:
        # tell simulation it can shut down
        env.endSimulation()
    
# set up the socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 50007))

# plan
planWithMorse(sock)
sock.shutdown(socket.SHUT_RDWR)
sock.close()


