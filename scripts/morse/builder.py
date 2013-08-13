# MORSE builder script

import logging
import os
import subprocess
import sys

import bpy

import morse.builder

OMPL_DIR=os.path.dirname(__file__)
GOALSTRINGS=['.goalLocRot','.goalRot','.goalRegion']

# Determine the mode to use (third argument)
mode = sys.argv[sys.argv.index('--') + 3]

# Hide the Blender window if we're planning
winID = subprocess.check_output(['bash', '-c',
    'wmctrl -l | grep morse_default_autorun | awk \'{ print $1 }\'']).decode()[:-1]

if mode == 'PLAN':
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -T "OMPL MORSE Planner"' % winID])
elif mode == 'PLAY':
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -T "OMPL MORSE Player"' % winID])
elif mode == 'QUERY':
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -T "OMPL MORSE Control Query"' % winID])

if mode == 'PLAN' or mode == 'QUERY':
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -b add,shaded' % winID])
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -e 0,100,100,600,200' % winID])

# Disable logging of socket communication because there will be a lot of it
sockloggers = (logging.getLogger("morse.morse.core.request_manager"),
               logging.getLogger("morse.morse.middleware.socket_request_manager"))
sockloggers[0].setLevel(logging.ERROR)
sockloggers[1].setLevel(logging.ERROR)

# Load the .blend file (first argument after '--')
envpath = sys.argv[sys.argv.index('--') + 1]
print("\n* Loading scene <%s>.\n" % envpath)
env = morse.builder.Environment(envpath)

# Replace the robot(s)
to_delete = []
i = 0
for obj in bpy.context.scene.objects:
    # if this object has the marks of a robot, but not a goal
    if obj.game.properties.get('RobotType') and \
        not [True for goalStr in GOALSTRINGS if obj.name.endswith(goalStr)]:
        rtype = obj.game.properties['RobotType'].value
        ctype = obj.game.properties['ControllerType'].value
        pos = obj.location
        rot = obj.rotation_euler
        # avoid name collision and mark for deletion
        rname = obj.name
        rnameSafe = rname.replace('.','_')
        if rname != rnameSafe:
            print("WARNING: had to rename robot %s to %s because dots not allowed in MORSE names"
                  % (rname, rnameSafe))
            for goalStr in GOALSTRINGS:
                goal = bpy.context.scene.objects.get(obj.name + goalStr)
                if goal:
                    print("   ^     also renamed goal %s" % goal.name)
                    goal.name = rnameSafe + goalStr
            rname = rnameSafe
        obj.name += '_'
        to_delete.append(obj)
        # add the MORSE components
        robot = getattr(morse.builder, rtype)(rname)
        motion = getattr(morse.builder, ctype)(robot.name+'Motion')
        # copy pose
        robot.location = pos
        robot.rotation_euler = rot
        # necessary for wheeled robots:
        if isinstance(robot, morse.builder.WheeledRobot):
            wheels = [child.name for child in robot._bpy_object.children if "wheel" in child.name.lower()]
            wheels.sort()
            while len(wheels) < 4:  # some don't have all 4 wheels
                wheels.append(None)
            robot.properties(WheelFLName = wheels[0], WheelFRName = wheels[1],
                WheelRLName = wheels[2], WheelRRName = wheels[3])
            robot.unparent_wheels()
        robot.append(motion)    # port = 4000
        motion.add_service('socket')
        i += 1
# delete the stand-in models
for obj in to_delete:
    bpy.context.scene.objects.unlink(obj)
# disallow sleeping
for obj in bpy.context.scene.objects:
    if obj.game.physics_type == 'RIGID_BODY':
            obj.game.use_sleep = True   # backwards; it means "no sleeping"

"""
robot = morse.builder.SegwayRMP400()
robot.unparent_wheels()
motion = morse.builder.MotionVWDiff()
robot.append(motion)
motion.add_service('socket')  """  

# Fix the timestep physics is advanced by each frame
#bpy.context.scene.game_settings.fps = 60
#bpy.context.scene.game_settings.use_frame_rate = True
#bpy.context.scene.game_settings.restrict_animation_updates = True
bpy.context.scene.game_settings.show_framerate_profile = True

# Create '__planner' object so we can set up the game engine
bpy.ops.object.add()
obj = bpy.context.object
obj.name = "__planner"

# Determine the solution path file to use (second argument)
outpath = sys.argv[sys.argv.index('--') + 2]
bpy.ops.object.game_property_new(type='STRING', name="Outpath")
obj.game.properties['Outpath'].value = outpath

# Set the mode setting
bpy.ops.object.game_property_new(type='STRING', name="Mode")
obj.game.properties['Mode'].value = mode

# Mode specific configuration:
if mode == 'PLAY':
    bpy.context.scene.game_settings.use_animation_record = True

# Add 'Tick' sensor
bpy.ops.logic.sensor_add(type='DELAY', name='Tick')
tick = obj.game.sensors['Tick']
tick.use_repeat = True

# Add 'communicator.py' text block
bpy.ops.text.open(filepath=OMPL_DIR + "/communicator.py")

# Add 'Comm' controller
bpy.ops.logic.controller_add(type='PYTHON', name='Comm')
comm = obj.game.controllers['Comm']
comm.mode = 'MODULE'
comm.module = 'communicator.main'

# Link Tick with Comm so it's run every frame
tick.link(comm)

env.create()



