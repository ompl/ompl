# plugin.py
# To be run within Blender when the Game Engine starts; spawns a
#  Python script outside of Blender and provides a method of
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
    
    
# Procedures to be called by external script; each one
#  must write a newline-terminated response string to stdin
#  that can be eval()'ed; also each one must return True
#  if the main while loop should continue running

external = None # will be initialized by main()

def quit():
    
    # null response
    external.stdin.write(b'None\n')
    
    # signal to exit loop
    return False


def extractState():
    
    # generate string from state without newlines
    stateStr = stringify(getState())
    
    # respond with encoded state string
    external.stdin.write(stateStr.encode() + b'\n')
    
    return True


def submitState(state):

    # load the state into the Game Engine
    setState(state)
    
    # null response
    external.stdin.write(b'None\n')
    
    return True


# Plugin's main function

def main():

    # external script's stderr file
    debugOut = open(OMPL_DIR + '/scripts/morse/ext.out','w')
    
    global external
    external = subprocess.Popen(
        OMPL_DIR + '/scripts/morse/external.py',
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=debugOut)
    
    cmd = 'True'
    
    # execute each command until one returns False
    while eval(cmd):
        
        # retrieve the next command
        cmd = external.stdout.readline().decode('utf-8')[:-1]
        print('received command: ' + cmd)
    
    debugOut.close()


main()



