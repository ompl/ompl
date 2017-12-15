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

# To be run within the Blender game engine; spawns an OMPL
#  planner script outside of Blender and provides a method of
#  extracting and submitting data to the Blender simulation by
#  the external script

import sys
import subprocess
import inspect
import socket
import pickle

import bpy
import bge
import mathutils

import ompl.morse

OMPL_DIR = ompl.morse.__path__[0]

## \brief List of rigid body Blender objects
rigidObjects = []

## \brief List of tuple pairs of objects acting as goal specifications
#     and their game engine counterparts
goalObjects = []

## \brief Subset of goalObjects acting as goal region specifications
goalRegionObjects = []

## \brief Socket to talk to the MyEnvironment object
sock = None

## \brief Used by main() to delay 1 second until MORSE is initialized
tickcount = -60

##
# \brief Returns the state tuple for an object, consisting of position,
#    linear velocity, angular velocity, and orientation
def getObjState(gameobj):

    # Convert vectors and matrices to tuples before returning
    return (gameobj.worldPosition.to_tuple(),
            gameobj.worldLinearVelocity.to_tuple(),
            gameobj.worldAngularVelocity.to_tuple(),
            tuple(gameobj.worldOrientation.to_quaternion()))

##
# \brief Returns the state tuple for an object, consisting only of
#    position and orientation
def getGoalLocRotState(gameobj):

    return (gameobj.worldPosition.to_tuple(),
            tuple(gameobj.worldOrientation.to_quaternion()))

##
# \brief Returns the state tuple for an object, consisting only of
#    orientation
def getGoalRotState(gameobj):

    return tuple(gameobj.worldOrientation.to_quaternion())

##
# \brief Returns the state tuple consisting only of location
def getGoalRegionState(gameobj):

    return gameobj.worldPosition.to_tuple()

##
# \brief Sets the state for a game object from a tuple consisting of position,
#    linear velocity, angular velocity, and orientation
def setObjState(gameobj, oState):

    gameobj.worldPosition = oState[0]
    gameobj.worldLinearVelocity = oState[1]
    gameobj.worldAngularVelocity = oState[2]
    gameobj.worldOrientation = mathutils.Quaternion(oState[3])

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

# #
# Procedures to be called by planner script; each one
#   must write a response string to the socket
#   that can be eval()'ed; also each one must return True
#   if the communicate() while loop should continue running.
# #

##
# \brief Return a list of tuples explaining the criteria for a goal state:
#    [(index of body in state space, (loc,rot) OR rot, locTol[, rotTol]), ...]
def getGoalCriteria():

    crit = []
    rigidNames = list(map(lambda o: o.name, rigidObjects))

    # For each goal object
    for (gbody, gameobj) in goalObjects:
        try:
            # Which rigid body does it correspond to?
            j = gbody.name.rfind('.')
            i = rigidNames.index(gbody.name[:j])

            if gbody.name.endswith('.goalPose'):
                # If it's a pose, use location and rotation
                crit.append((i, getGoalLocRotState(gameobj), gbody['locTol'], gbody['rotTol']))

            elif gbody.name.endswith('.goalRot'):
                # If it's a rotationm use rotation only
                crit.append((i, getGoalRotState(gameobj), gbody['rotTol']))

            else:
                # If it's a region, get location only
                crit.append((i, getGoalRegionState(gameobj)))

        except ValueError:
            # Goal specification for non-existent body
            print("Ignoring stray goal criterion %s" % gbody.name)

        # Non-region objects don't need to stick around during simulation
        if not gameobj in goalRegionObjects:
            gameobj.endObject()

    # Send the pickled description of goal criteria
    sock.sendall(pickle.dumps(crit))

    return True

##
# \brief Return True if all .goalRegion sensors are in collision with
#    or inside of their respective bodies
def goalRegionSatisfied():

    # For each goal region
    for obj in goalRegionObjects:

        # Check for collision
        sensor = obj.sensors["__collision"]
        if not sensor.hitObject:
            # If no collision, check if it's entirely inside using a ray cast
            (hit, point, normal) = obj.rayCast(obj, \
                bge.logic.getCurrentScene().objects[sensor.propName], \
                0, sensor.propName, 1, 1, 0)
            # If we're on the inside, the first face we hit should be facing away from us
            if not hit or sum(normal[i]*(obj.worldPosition[i]-point[i]) for i in range(3)) > 0:
                return False
    return True

##
# \brief Discover the motion controller services and how to call them; also find the
#    control dimension and the control bounds
#    Returns [sum_of_Nargs, [cbm0, cbM0, ...], (component_name,service_name,Nargs), ...]
def getControlDescription():

    settings = bpy.context.scene.objects['ompl_settings']
    desc = [0, []]
    # Query the request_manager for a list of services
    for name, inst in bge.logic.morsedata.morse_services.request_managers().items():
        if name == 'morse.middleware.socket_request_manager.SocketRequestManager':
            for cname, services in inst.services().items():
                if cname.endswith('Motion'):
                    for svc in services:
                        if svc == 'set_property':
                            continue
                        # Add info to the description
                        n = len(inspect.getargspec(inst._services[cname, svc][0])[0]) - 1
                        if n == 0: # Services like stop() aren't really helpful to OMPL
                            continue
                        # Fill it in backwards
                        desc = desc[:2] + [(cname, svc, n)] + desc[2:]
                        desc[0] += n

    # Fill in the control bounds at the beginning
    for i in range(min(16, desc[0])):
        desc[1] += [settings['cbm%i'%i], settings['cbM%i'%i]]

    # Send the encoded description
    sock.sendall(pickle.dumps(desc))

    return True

##
# \brief Return the number of rigid bodies and their positional bounds
def getRigidBodiesBounds():

    # Check whether user set the autopb flag
    settings = bpy.context.scene.objects['ompl_settings']
    if settings['autopb']:

        # Find min and max values for all objects' bounding box vertices
        mX = mY = mZ = float('inf')
        MX = MY = MZ = float('-inf')
        for gameobj in bge.logic.getCurrentScene().objects:
            # Look through all the objects
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

        # Ioan's formula to get reasonable positional bounds:
        dx = MX-mY
        dy = MY-mY
        dz = MZ-mZ
        dM = max(dx, dy, dz)
        dx = dx/10.0 + dM/100.0
        dy = dy/10.0 + dM/100.0
        dz = dz/10.0 + dM/100.0
        mX -= dx
        MX += dx
        mY -= dy
        MY += dy
        mZ -= dz
        MZ += dz

        print("OMPL: Inferred position bounds [[%f,%f],[%f,%f],[%f,%f]]"
              % (mX, MX, mY, MY, mZ, MZ))
    else:

        # Use user-specified positional bounds
        mX = settings['pbx']
        MX = settings['pbX']
        mY = settings['pby']
        MY = settings['pbY']
        mZ = settings['pbz']
        MZ = settings['pbZ']

    # Get linear and angular velocity bounds
    lb = [settings['lbm'], settings['lbM']]
    lb += lb + lb
    ab = [settings['abm'], settings['abM']]
    ab += ab + ab

    # Collect all the information and send it
    bounds = [len(rigidObjects), [mX, MX, mY, MY, mZ, MZ], lb, ab]
    sock.sendall(pickle.dumps(bounds))

    return True

##
# \brief Helper to recursively delete an object and all its children
def _recurseDelete(obj):

    # Call this function on all the children
    for child in obj.children[:]:
        _recurseDelete(child)

    # Select only this object and delete it
    bpy.ops.object.select_pattern(pattern=obj.name, case_sensitive=True, extend=False)
    bpy.ops.object.delete()

##
# \brief Close the socket and tell Blender to stop the game engine
def endSimulation():

    global sock

    # null response
    sock.sendall(b'\x06')

    # Close the socket
    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
    sock = None

    # Stop the game engine loop
    bge.logic.endGame()

    mode = bpy.data.objects['ompl_settings']['Mode']
    if mode == 'PLAY':

        # We need to clean up the created animation file
        animpath = bpy.data.objects['ompl_settings']['Animpath']
        animtmppath = animpath + ".tmp"

        # Prevent autostart
        bpy.context.scene.game_settings.use_auto_start = False

        # Mark unwanted objects for deletion; set the active camera
        toDelete = []
        for obj in bpy.context.scene.objects:
            if obj.name in ['Scene_Script_Holder', 'CameraFP',
                            'ompl_settings', 'MORSE.Properties',
                            '__morse_dt_analyser']:
                toDelete.append(obj)
            elif obj.name == 'Camera':
                bpy.context.scene.camera = obj
            for child in obj.children:
                if child.name.endswith('Motion'):
                    toDelete.append(child)
        # Delete them and their children
        for obj in toDelete:
            _recurseDelete(obj)

        # Delete text files in the blend file.
        for txt in ['communicator.py', 'component_config.py',
                    'setup_path.py', 'Info.txt']:
            txt = bpy.data.texts.get(txt)
            if txt:
                bpy.data.texts.remove(txt)

        # Save animation curves to tmp file
        print("OMPL: Writing to tmp file '" + animtmppath + "'")
        bpy.ops.wm.save_mainfile(filepath=animtmppath, check_existing=False)

    # Signal to exit loop
    return False

##
# \brief Stop the communication loop and advance to the next tick
def nextTick():

    # null response
    sock.sendall(b'\x06')

    # Signal to exit communication loop
    return False

##
# \brief Retrieve a list of state tuples for all rigid body objects
def extractState():

    # Generate state list (and include the goalRegionSat flag)
    state = list(map(getObjState, rigidObjects)) + [int(goalRegionSatisfied())]

    # Pickle and send it
    sock.sendall(pickle.dumps(state))

    return True

##
# \brief Load position, orientation, and velocity data into the Game Engine.
#    Input data is a list of object states, ordered just like the
#    state list returned by extractState()
def submitState():

    # Unpickle the state
    sock.sendall(b'\x06')
    s = unpickleFromSocket(sock)
    sock.sendall(b'\x06')

    # Load the state into the Game Engine, one object at a time
    for (i, state) in enumerate(s):
        setObjState(rigidObjects[i], state)

    return True


# #
# High-level functions to manage logic and communication
# #

##
# \brief Run once the game engine is started, after MORSE is initialized;
#    spawns the external Python script 'planner.py'
def spawn_planner():

    # Set up the server socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('localhost', 50007))

    mode = bpy.data.objects['ompl_settings']['Mode']
    if mode == 'PLAN':
        # Spawn planner.py
        f = '/planner.py'
    elif mode == 'PLAY':
        # Spawn player.py
        f = '/player.py'

    if mode != 'QUERY':
        # Pass the name of the output (or input) file
        subprocess.Popen(['env', sys.executable, '-P', OMPL_DIR + f, '--', \
            bpy.data.objects['ompl_settings']['Outpath']])

    # Make a connection
    s.listen(0)
    global sock
    sock, _ = s.accept()

##
# \brief This function is run during MORSE simulation between every
#    tick; provides a means of servicing requests from planner.py
def communicate():

    # Evaluation of cmd determines whether to continue looping
    cmd = 'True'
    global sock
    try:
        while eval(cmd):

            # Retrieve the next command
            cmd = sock.recv(32).decode('utf-8')   # commands are shorter than 32 bytes

            if cmd == '':
                # Something is wrong, shut down the game engine
                sock.close()
                sock = None
                bge.logic.endGame()
                break

    except Exception as msg:
        # Error 104 indicates crash and meaningful traceback happened in a different script
        if str(msg) != '[Errno 104] Connection reset by peer':
            # If it was something else, then raise the exception
            raise

##
# \brief Called by the Blender logic system once per tick; spawns the planner
#    after simulation has been running for 1 second and begin communication
def main():

    # 1 second has passed when tickcount hits 0 (started at -60)
    global tickcount
    tickcount += 1
    if tickcount < 0:
        return

    if tickcount == 0:

        # Build the lists of rigid body objects and goal objects
        global rigidObjects
        global goalObjects
        global goalRegionObjects
        print("\033[93;1mGathering list of rigid bodies and goal criteria:")

        scn = bge.logic.getCurrentScene()
        objects = scn.objects
        for gameobj in sorted(objects, key=lambda o: o.name):

            # Get the Blender object for this game body, if there is one
            obj = bpy.data.objects.get(gameobj.name)
            if not obj:
                continue

            # Check if it's named as a goal criterion
            if [True for goalStr in ['.goalPose', '.goalRegion', '.goalRot'] if gameobj.name.endswith(goalStr)]:
                print("\t> goal criterion " + gameobj.name)

                if gameobj.name.endswith('.goalRegion'):
                    # Make sure the corresponding body is linked to this collision sensor
                    body = bge.logic.getCurrentScene().objects.get(obj.name[:-11])
                    if not body:
                        continue
                    body[body.name] = True
                    goalRegionObjects.append(gameobj)

                goalObjects.append((obj, gameobj))

            elif obj.game.physics_type == 'RIGID_BODY':

                # Check if it's not a goal or child of a goal, but is a rigid body
                parent = gameobj.parent
                goal_child = False
                while parent:
                    if [True for goalStr in ['.goalPose', '.goalRegion', '.goalRot'] \
                      if parent.name.endswith(goalStr)]:
                        goal_child = True
                        break
                    parent = parent.parent

                if not goal_child:
                    print("[%i] rigid body %s" % (len(rigidObjects), gameobj.name))
                    rigidObjects.append(gameobj)

        # Reset terminal colors
        print('\033[0m')

        # Start the external planning script
        spawn_planner()

    if sock:
        # Handle requests from the planner
        communicate()
