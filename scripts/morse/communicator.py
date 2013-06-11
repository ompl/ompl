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

bge.logic.setMaxLogicFrame(1)
bge.logic.setMaxPhysicsFrame(1)
bge.logic.setLogicTicRate(60)
bge.logic.setPhysicsTicRate(60)

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
    

rigidObjects = []   # initialized in main()

def setState(state):

    for i in range(len(state)):
        
        # set its state
        setObjState(rigidObjects[i], state[i])


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

planner = None # initialized by spawn_planner()

def endPlanning():
    
    # null response
    planner.stdin.write(b'None\n')
    
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
    """
    Retrieve a list of state tuples for all rigid body objects,
    sorted by object name.
    """
    
    # generate state list
    state = list(map(getObjState, rigidObjects))
    
    # respond with encoded state string
    planner.stdin.write(stringify(state).encode() + b'\n')
    
    return True


def submitState(state):
    """
    Load position, orientation, and velocity data into the Game Engine.
    Input is a list of object states sorted by name, just like the
    format returned by extractState().
    """

    # load the state into the Game Engine
    for i in range(len(state)):
        
        # set each object's state
        setObjState(rigidObjects[i], state[i])
    
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


tickcount = -60

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
        if cmd != 'nextTick()':
            print('At tick %i, received command: %s' % (tickcount, cmd))


def main():
    """
    Decides whether to spawn the planner or communicate with an
    existing one.
    """

    # Wait a second for MORSE to finish initializing
    global tickcount
    tickcount += 1
    if tickcount < 0:
        return

    # After planner is spawned, game property 'spawned' will be True
    
    props = bpy.context.scene.objects['__planner'].game.properties
    if not props['spawned'].value:
        
        # build the sorted list of rigid body objects
        global rigidObjects
        print("Gathering list of rigid bodies:")
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
                    
        # start the external planning script and service it
        spawn_planner()
        props['spawned'].value = True
        communicate()
        
    else:
        
        # handle requests from the existing planner
        communicate()



