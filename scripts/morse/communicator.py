# communicator.py
# To be run within the Blender game engine; spawns an OMPL
#  planner script outside of Blender and provides a method of
#  extracting and submitting data to the Blender simulation by
#  the external script

import os
import subprocess
import inspect
import socket
import pickle

import bpy
import bge
import mathutils

OMPL_DIR=os.path.dirname(__file__)
GOALSTRINGS=['.goalLocRot','.goalRot','.goalRegion']

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

def getGoalLocRotState(gameobj):
    """
    Returns the state tuple for an object, consisting only of
    position and orientation.
    """
    
    # convert Vectors and Matrices to tuples before returning
    return (gameobj.worldPosition.to_tuple(),
            tuple(gameobj.worldOrientation.to_quaternion()))

def getGoalRotState(gameobj):
    """
    Returns the state tuple for an object, consisting only of
    orientation.
    """
    
    # convert to tuple before returning
    return tuple(gameobj.worldOrientation.to_quaternion())

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
    Return a list of tuples explaining the criteria for a goal state:
     [(index of body in state space, (loc,rot) | rot, locTol[, rotTol]), ...]
    Also destroys goal objects that are not regions since they will no longer be needed.
    """
    crit = []
    for gbody in goalObjects:
        try:
            # which rigid body does this goal body correspond to?
            j = gbody.name.rfind('.')
            i = list(map(lambda o: o.name, rigidObjects)).index(gbody.name[:j])
            if gbody.name.endswith('.goalLocRot'):
                crit.append((i,getGoalLocRotState(gbody),gbody['locTol'],gbody['rotTol']))
            elif gbody.name.endswith('.goalRot'):
                crit.append((i,getGoalRotState(gbody),gbody['rotTol']))
        except ValueError:
            print("Ignoring stray goal criterion %s" % gbody.name)
        
        # delete the goal object if it's not a region goal
        if not gbody.name.endswith('.goalRegion'):
            gbody.endObject()
    
    # send the pickled response
    sock.sendall(pickle.dumps(crit))
    
    return True

def getControlDescription():
    """
    Discover the motion controller services and how to call them; also finds the
    control dimension and the control bounds.
    Returns [sum_of_nargs, [cbm0, cbM0, ...], (component_name,service_name,nargs), ...]
    """
    settings = bpy.context.scene.objects['__settings'].game.properties
    desc = [0, []]
    # query the request_manager for a list of services
    for name, inst in bge.logic.morsedata.morse_services.request_managers().items():
        if name == 'morse.middleware.socket_request_manager.SocketRequestManager':
            for cname, services in inst.services().items():
                if cname.endswith('Motion'):
                    for svc in services:
                        # add info to the description
                        n = len(inspect.getargspec(inst._services[cname,svc][0])[0]) - 1  # exclude self arg
                        if n > 0:   # services like stop() aren't really helpful to OMPL
                            desc = desc[:2] + [(cname, svc, n)] + desc[2:]  # fill it in backwards
                            desc[0] += n
    
    # fill in the control bounds
    for i in range(min(16,desc[0])):
        desc[1] += [settings['cbm%i'%i].value, settings['cbM%i'%i].value]
    
    # send the encoded list
    sock.sendall(pickle.dumps(desc))
    
    return True

def getRigidBodiesBounds():
    """
    Return the number of rigid bodies and positional bounds for them.
    """
    # Check whether user set the autopb flag
    settings = bpy.context.scene.objects['__settings'].game.properties
    if settings['autopb'].value:
        # Find min and max values for all objects' bound box vertices
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
        
    else:
        # Use user-specified positional bounds
        mX = settings['pbx'].value
        MX = settings['pbX'].value
        mY = settings['pby'].value
        MY = settings['pbY'].value
        mZ = settings['pbz'].value
        MZ = settings['pbZ'].value
    
    # Get lin and ang bounds
    lb = [settings['lbm'].value, settings['lbM'].value]
    lb += lb + lb
    ab = [settings['abm'].value, settings['abM'].value]
    ab += ab + ab
    
    # gather the information
    bounds = [len(rigidObjects), [mX, MX, mY, MY, mZ, MZ], lb, ab]
    
    # send the pickled list
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
    
    # unfreeze time
    #bge.logic.freezeTime(False)
    
    bge.logic.endGame()
    
    mode = bpy.data.objects['__planner'].game.properties['Mode'].value
    
    if mode == 'PLAY':
        # Clean up:
        
        # no autostart
        bpy.context.scene.game_settings.use_auto_start = False
        
        # remove unwanted objects
        for obj in bpy.context.scene.objects[:]:
            if obj.name in ['__planner','Scene_Script_Holder']:
                bpy.context.scene.objects.unlink(obj)
                
        # save animation curves to file
        # TODO let user pick the file
        bpy.ops.wm.save_mainfile(filepath="/home/caleb/repos/ompl_morse/scripts/morse/iposave.blend",
                                 check_existing=False)
    
    # signal to exit loop
    return False

def stepRes(res):
    """
    Set propagate tics per world step.
    """
    # null response
    sock.sendall(b'\x06')
    
    bge.logic.setPropagateTics(int(res*60))
    
    return True

def nextTick():
    """
    Stop the communicate() while loop to advance to the next tick.
    """
    
    # null response
    sock.sendall(b'\x06')
    
    # signal to exit communication loop
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
    
    # freeze time, so only nextTick() can advance it
    #bge.logic.freezeTime(True)
    
    mode = bpy.data.objects['__planner'].game.properties['Mode'].value
    
    if mode == 'PLAN':
        # spawn planner.py
        f = '/planner.py'
    elif mode == 'PLAY':
        # spawn player.py
        f = '/player.py'
    
    if mode != 'QUERY':
        # pass the name of the output (or input) file
        subprocess.Popen([OMPL_DIR + f, bpy.data.objects['__planner'].game.properties['Outpath'].value])
    
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
    global sock
    # execute each command until one returns False
    try:
        while eval(cmd):
            # retrieve the next command
            cmd = sock.recv(32).decode('utf-8')   # commands are very short
            #print(cmd)
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
                print("rigid body " + gameobj.name)
                rigidObjects.append(gameobj)
            
            # check if it's a goal criterion
            elif [True for goalStr in GOALSTRINGS if gameobj.name.endswith(goalStr)]:
                print("goal state " + gameobj.name)
                goalObjects.append(gameobj)
        
        print('\033[0m')
        
        # start the external planning script
        spawn_planner()

    if sock and bge.logic.getPropagateTics()==0:
        # handle requests from the planner
        communicate()



