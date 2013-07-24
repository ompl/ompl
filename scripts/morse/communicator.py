# communicator.py
# To be run within the Blender game engine; spawns an OMPL
#  planner script outside of Blender and provides a method of
#  extracting and submitting data to the Blender simulation by
#  the external script

# IMPORTANT! Set this manually for now
OMPL_DIR='/home/caleb/repos/ompl_morse'

import subprocess
import inspect
import socket
import pickle

import bpy
import bge
import mathutils

import morse.builder
import morse.core

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

def getGoalState(gameobj):
    """
    Returns the state tuple for an object, consisting of position,
    linear velocity, angular velocity, and orientation
    """
    
    # convert Vectors and Matrices to tuples before returning
    return (gameobj.worldPosition.to_tuple(),
            (0.0,0.0,0.0),
            (0.0,0.0,0.0),
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

# Procedures to be called by planner script; each one
#  must write a response string to the socket
#  that can be eval()'ed; also each one must return True
#  if the communicate() while loop should continue running.

goalObjects = []    # initialized in main()
sock = None # initialized in spawn_planner()

def getGoalCriteria():
    """
    Return a list of tuples explaining the criteria for a goal state.
    """
    crit = []
    for gbody in goalObjects:
        try:
            # which rigid body does this goal body correspond to?
            i = list(map(lambda o: o.name, rigidObjects)).index(gbody.name[:-5])
            crit.append((i,getGoalState(gbody)))
        except ValueError:
            print("Ignoring goal criterion for non-existant or non-rigid-body object: " + gbody.name[:-5])
    
    # send the pickled response
    sock.sendall(pickle.dumps(crit))
    
    return True

def getControlDescription():
    """
    Discover the motion controller services and how to call them.
    Returns [sum_of_nargs, (component_name,service_name,nargs), ...]
    """
    desc = [0]
    # query the request_manager for a list of services
    for name, inst in bge.logic.morsedata.morse_services.request_managers().items():
        if name == 'morse.middleware.socket_request_manager.SocketRequestManager':
            for cname, services in inst.services().items():
                if cname.startswith('motion_'):
                    for svc in services:
                        # add info to the description
                        n = len(inspect.getargspec(inst._services[cname,svc][0])[0]) - 1  # exclude self arg
                        if n > 0:   # services like stop() aren't really helpful to OMPL
                            desc.append((cname, svc, n))
                            desc[0] += n
    # send the encoded list
    sock.sendall(pickle.dumps(desc))
    
    return True

def getRigidBodiesBounds():
    """
    Return the number of rigid bodies and positional bounds for them.
    """
    #TODO user may need to override this
    
    # find min and max values for all objects' bound box vertices
    mX = mY = mZ = float('inf')
    MX = MY = MZ = float('-inf')
    for gameobj in bge.logic.getCurrentScene().objects:
        obj = bpy.data.objects.get(gameobj.name)
        if not obj:
            continue
            
        box = obj.bound_box
        mX = min(mX, min(box[i][0] + obj.location[0] for i in range(8)))
        mY = min(mY, min(box[i][1] + obj.location[1] for i in range(8)))
        mZ = min(mZ, min(box[i][2] + obj.location[2] for i in range(8)))
        MX = max(MX, max(box[i][1] + obj.location[0] for i in range(8)))
        MY = max(MY, max(box[i][2] + obj.location[1] for i in range(8)))
        MZ = max(MZ, max(box[i][0] + obj.location[2] for i in range(8)))
    
    # Ioan's formula:
    dx = MX-mY
    dy = MY-mY
    dz = MZ-mZ
    dM = max(dx,dy,dz)
    dx = dx/10.0 + dM/100.0
    dy = dy/10.0 + dM/100.0
    dz = dz/10.0 + dM/100.0
    mX -= dx
    MX += dx
    mY -= dy
    MY += dy
    mZ -= dz
    MZ += dz
    
    # make safe for eval()
    bounds = [len(rigidObjects), [mX, MX, mY, MY, mZ, MZ]]
    
    # send the encoded list
    sock.sendall(pickle.dumps(bounds))
    
    return True

def endSimulation():
    """
    Close the socket and tell Blender to stop the game engine.
    """
    
    global sock # we're going to modify it
    
    # null response
    sock.sendall(b'\x06')
    
    # close the socket
    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
    
    sock = None
    
    # shutdown the game engine
    bge.logic.endGame()
    
    # signal to exit loop
    return False

captureNextFrame = False

def nextTick(framecapture=False):
    """
    Stop the communicate() while loop to advance to the next tick.
    """
    
    # null response
    sock.sendall(b'\x06')
    
    if framecapture:
        global captureNextFrame
        captureNextFrame = True
    
    # signal to exit loop
    return False

def extractState():
    """
    Retrieve a list of state tuples for all rigid body objects.
    """
    # generate state list
    state = list(map(getObjState, rigidObjects))
    
    # pickle and send it
    sock.sendall(pickle.dumps(state))
    
    return True

def submitState():
    """
    Load position, orientation, and velocity data into the Game Engine.
    Input is a list of object states, ordered just like the
    state list returned by extractState().
    """
    # ready to receive pickle
    sock.sendall(b'\x06')
    
    # unpickle the state
    s = unpickleFromSocket(sock)
    
    # null response
    sock.sendall(b'\x06')
    
    # load the state into the Game Engine
    for i in range(len(s)):
        
        # set each object's state
        setObjState(rigidObjects[i], s[i])
    
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
    
    settings = bpy.data.objects['__planner'].game.properties
    
    if settings['Mode'].value == 'PLAN':
    
        #bge.logic.setTimeMultiplier(16)
        # spawn planner.py
        f = '/scripts/morse/planner.py'
        
    elif settings['Mode'].value == 'PLAY':
    
        #bge.logic.setTimeMultiplier(1)
        # spawn player.py
        f = '/scripts/morse/player.py'
        
    else:
        print('Unrecognized mode setting!')
        return
    
    # pass the name of the output (or input) file
    subprocess.Popen([OMPL_DIR + f, bpy.data.objects['__planner'].game.properties['Outpath'].value])
            
    # make a connection
    s.listen(0)
    global sock
    sock, addr = s.accept()


tickcount = -60 # used by main() to wait until MORSE is initialized
framecount = 0

def communicate():
    """
    This function is run during MORSE simulation between every
    tick; provides a means of servicing requests from planner.py.
    """

    # Capture recording
    global captureNextFrame
    global framecount
    if captureNextFrame:
        bpy.ops.screen.screenshot(filepath='/home/caleb/avi/%04i.jpg' % framecount)
        captureNextFrame = False
        framecount += 1
    
    cmd = 'True'

    # execute each command until one returns False
    global sock
    try:
        while eval(cmd):
            # retrieve the next command
            cmd = sock.recv(32).decode('utf-8')   # commands are very short
            if cmd == '':
                # close the socket
                sock.close()
                sock = None
                # shutdown the game engine
                bge.logic.endGame()
                break
    except Exception as msg:
        # crash and traceback happen elsewhere with Errno 104
        if str(msg) != '[Errno 104] Connection reset by peer':
            raise

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
        # pace the simulation so that Blender doesn't try to speed up
        #  even if it thinks it's falling behind
        bge.logic.setMaxLogicFrame(1)
        bge.logic.setMaxPhysicsFrame(1)
        
        # build the lists of rigid body objects and goal objects
        global rigidObjects
        global goalObjects
        print("\033[93;1mGathering list of rigid bodies and goal criteria:")
        scn = bge.logic.getCurrentScene()
        objects = scn.objects
        for gameobj in sorted(objects, key=lambda o: o.name):
            
            # get the corresponding Blender object, if there is one
            obj = bpy.data.objects.get(gameobj.name)
            if not obj:
                continue
            
            # check if it's a rigid body
            if obj.game.physics_type == 'RIGID_BODY':
                print("rigid " + gameobj.name)
                rigidObjects.append(gameobj)
            
            # check if it's a goal criterion
            elif gameobj.name.endswith('.goal'):
                print("goal " + gameobj.name)
                goalObjects.append(gameobj)
        
        print('\033[0m')
        
        # start the external planning script
        spawn_planner()

    if sock:
        # handle requests from the planner
        communicate()



