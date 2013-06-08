# communicator.py
# To be run within Blender when the game engine starts; can spawn
#  planner script outside of Blender and provides a method of
#  extracting and submitting data to the Blender simulation by
#  an external program

# IMPORTANT! Set this manually for now
OMPL_DIR='/home/caleb/repos/ompl_morse'

import subprocess

import bpy
import bge
import mathutils


# Routines for accessing Blender internal data

def getObjState(gameobj):
    """
    Returns the state tuple for an object, consisting of position,
    orientation, linear velocity, and angular velocity
    """
    
    # convert Vectors and Matrices to tuples before returning
    return (gameobj.worldPosition.to_tuple(),
            tuple(gameobj.worldOrientation.to_quaternion()),
            gameobj.worldLinearVelocity.to_tuple(),
            gameobj.worldAngularVelocity.to_tuple())
            
            
def setObjState(gameobj, oState):
    """
    Sets the state for a game object from a tuple consisting of position,
    orientation, linear velocity, and angular velocity
    """
    
    gameobj.worldPosition = oState[0]
    gameobj.worldOrientation = mathutils.Quaternion(oState[1])
    gameobj.worldLinearVelocity = oState[2]
    gameobj.worldAngularVelocity = oState[3]
    
    
def getState():
    """
    Retrieve a list of state tuples for all rigid body objects,
    sorted by object name.
    """

    # list of object states
    state = []
    
    # scene where all the objects are
    scn = bge.logic.getCurrentScene()
    
    # for each object in the game engine (sorted by name)
    for gameobj in sorted(scn.objects, key=lambda o: o.name):
        
        # get the corresponding Blender object, if there is one
        obj = bpy.data.objects.get(gameobj.name)
        if not obj:
            break
        
        # only get its state if it's a rigid body
        if obj.game.physics_type == 'RIGID_BODY':
            state.append(getObjState(gameobj))
    
    return state


def setState(state):
    """
    Load position, orientation, and velocity data into the Game Engine.
    Input is a list of object states sorted by name, just like in the
    above format.
    """
    
    # scene where all the objects are
    scn = bge.logic.getCurrentScene()
    
    # for each object in the game engine (sorted by name)
    for gameobj in sorted(scn.objects, key=lambda o: o.name):
        
        # get the corresponding Blender object, if there is one
        obj = bpy.data.objects.get(gameobj.name)
        if not obj:
            break
        
        # only set its state if it's a rigid body
        if obj.game.physics_type == 'RIGID_BODY':
            setObjState(gameobj, state.pop(0))


def stringify(thing):
    """
    Prepare a Python object for transmission over pipes.
    """
    
    # remove newlines from string representation
    return ' '.join(repr(thing).split('\n'))
    
    
# Procedures to be called by planner script; each one
#  must write a newline-terminated response string to stdin
#  that can be eval()'ed; also each one must return True
#  if the main while loop should continue running

planner = None # will be initialized by spawn_planner()

def endGame():
    
    # null response
    planner.stdin.write(b'None\n')
    
    # reset 'spawned' flag
    bpy.context.scene.camera.game.properties['spawned'].value = False
    
    # shutdown the game engine
    bge.logic.endGame()
    
    # signal to exit loop
    return False


def nextTick():
    
    # null response
    planner.stdin.write(b'None\n')
    
    # signal to exit loop
    return False


def extractState():
    
    # generate string from state without newlines
    stateStr = stringify(getState())
    
    # respond with encoded state string
    planner.stdin.write(stateStr.encode() + b'\n')
    
    return True


def submitState(state):

    # load the state into the Game Engine
    setState(state)
    
    # null response
    planner.stdin.write(b'None\n')
    
    return True


def spawn_planner():
    """
    Run when the game engine is started if planning is desired.
    Spawns the external Python script 'planner.py'.
    """
    
    # planner script's stderr file
    debugOut = open(OMPL_DIR + '/scripts/morse/plan.out','w')
    
    global planner  # helper functions above use this
    planner = subprocess.Popen(
        OMPL_DIR + '/scripts/morse/planner.py',
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=debugOut)
    
    debugOut.close()


def communicate():
    """
    This function is run during MORSE simulation between every
    physics tick; provides a means of accessing Blender game
    engine data
    """

    cmd = 'True'
    
    # execute each command until one returns False
    while eval(cmd):
        
        # retrieve the next command
        cmd = planner.stdout.readline().decode('utf-8')[:-1]
        print('received command: ' + cmd)
    


def main():
    """
    Decides whether to spawn the planner, communicate with an
    existing one, or do nothing.
    """

    # MORSE builder script will request the planner by setting
    # game property 'plan' to True; after planner is spawned,
    # game property 'spawned' will be True
    
    gameProps = bpy.context.scene.camera.game.properties
    
    if not gameProps['spawned'].value:
        if gameProps['plan'].value:
            
            # start the external planning script and service it
            spawn_planner()
            gameProps['spawned'].value = True
            communicate()
        
    else:
        # service requests from the existing planner
        communicate()



