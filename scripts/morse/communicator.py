# communicator.py
# To be run within the Blender game engine; spawns an OMPL
#  planner script outside of Blender and provides a method of
#  extracting and submitting data to the Blender simulation by
#  the external script

# IMPORTANT! Set this manually for now
OMPL_DIR='/home/caleb/repos/ompl_morse'

import subprocess
import socket

import bpy
import bge
import mathutils

# Set world step size to 1/60 s
bge.logic.setMaxLogicFrame(1)
bge.logic.setMaxPhysicsFrame(1)
bge.logic.setLogicTicRate(60)
bge.logic.setPhysicsTicRate(60)

# Routines for accessing Blender internal data

def getObjState(gameobj):
    """
    Returns the state tuple for an object, consisting of position,
    linear velocity, angular velocity, and orientation
    """
    
    # convert Vectors and Matrices to tuples before returning
    return (gameobj.worldPosition.to_tuple(),
            gameobj.worldLinearVelocity.to_tuple(),
            gameobj.worldAngularVelocity.to_tuple(),
            tuple(gameobj.worldOrientation.to_quaternion()))
            
            
def setObjState(gameobj, oState):
    """
    Sets the state for a game object from a tuple consisting of position,
    linear velocity, angular velocity, and orientation
    """
    
    gameobj.worldPosition = oState[0]
    gameobj.worldLinearVelocity = oState[1]
    gameobj.worldAngularVelocity = oState[2]
    gameobj.worldOrientation = mathutils.Quaternion(oState[3])
    

rigidObjects = []   # initialized in main()

def setState(state):

    for i in range(len(state)):
        
        # set its state
        setObjState(rigidObjects[i], state[i])
    
    
# Procedures to be called by planner script; each one
#  must write a response string to the socket
#  that can be eval()'ed; also each one must return True
#  if the communicate() while loop should continue running.

sock = None # initialized in spawn_planner()

def endSimulation():
    """
    Close the socket and tell Blender to stop the game engine.
    """
    
    # null response
    sock.sendall(b"None")
    
    # close the socket
    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
    
    global sock
    sock = None
    
    # shutdown the game engine
    bge.logic.endGame()
    
    # signal to exit loop
    return False


def nextTick():
    """
    Stop the communicate() while loop to advance to the next tick.
    """
    
    # null response
    sock.sendall(b"None")
    
    # signal to exit loop
    return False


def extractState():
    """
    Retrieve a list of state tuples for all rigid body objects.
    """
    
    # generate state list
    state = list(map(getObjState, rigidObjects))
    
    # respond with encoded state string
    sock.sendall(repr(state).encode())
    
    return True


def submitState(state):
    """
    Load position, orientation, and velocity data into the Game Engine.
    Input is a list of object states, ordered just like the
    state list returned by extractState().
    """

    # load the state into the Game Engine
    for i in range(len(state)):
        
        # set each object's state
        setObjState(rigidObjects[i], state[i])
    
    # null response
    sock.sendall(b"None")
    
    return True


# Functions to mangage communication

def spawn_planner():
    """
    Run once when the game engine is started, after MORSE is initialized.
    Spawns the external Python script 'planner.py'.
    """
    
    # set up the server socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('localhost', 50007))
    
    # spawn planner.py
    subprocess.Popen(OMPL_DIR + '/scripts/morse/planner.py')
    
    # make a connection
    s.listen(0)
    global sock
    sock, addr = s.accept()


tickcount = -60 # used by main() to wait until MORSE is initialized

def communicate():
    """
    This function is run during MORSE simulation between every
    tick; provides a means of servicing requests from planner.py.
    """

    cmd = 'True'

    # execute each command until one returns False
    while eval(cmd):
        
        # retrieve the next command
        while True:
            cmd = sock.recv(16384).decode('utf-8')   # TODO buffer size
            if cmd != '':
                break
        #if cmd != 'nextTick()':
        #print('\033[93;1mAt tick %i, received command: %s\033[0m' % (tickcount, cmd))


def main():
    """
    Spawn the planner when tickcount reaches 0. Communicate with an
    existing one when tickcount is positive.
    """

    # wait a second for MORSE to finish initializing
    global tickcount
    tickcount += 1
    if tickcount < 0:
        return
    
    if tickcount == 0:
        
        # build the list of rigid body objects
        global rigidObjects
        print("\033[93;1mGathering list of rigid bodies:")
        scn = bge.logic.getCurrentScene()
        objects = scn.objects
        for gameobj in sorted(objects, key=lambda o: o.name):
            
            # get the corresponding Blender object, if there is one
            obj = bpy.data.objects.get(gameobj.name)
            if not obj:
                continue
            
            # check if it's a rigid body
            if obj.game.physics_type == 'RIGID_BODY':
                print(gameobj.name)
                rigidObjects.append(gameobj)
        
        print('\033[0m')
        
        # start the external planning script
        spawn_planner()

    if sock:
        # handle requests from the planner
        communicate()



