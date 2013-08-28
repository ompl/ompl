# MORSE builder script

import logging
import os
import subprocess
import sys

import bpy

import morse.builder

OMPL_DIR=os.path.dirname(__file__)
GOALSTRINGS=['.goalPose','.goalRot','.goalRegion']

# Determine the mode to use (third argument)
mode = sys.argv[sys.argv.index('--') + 3]

# Hide the Blender window if we're planning
# (fails silently if wmctrl not installed)
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
    print(obj.name)
    # if this object has the marks of a robot, but not a goal
    if obj.get('RobotType') and \
        not [True for goalStr in GOALSTRINGS if obj.name.endswith(goalStr)]:
        rtype = obj['RobotType']
        ctype = obj['ControllerType']
        pos = obj.location
        rot = obj.rotation_euler
        # make names acceptable for MORSE
        rname = obj.name
        rnameSafe = rname.replace('.','_')
        if rname != rnameSafe:
            print("WARNING: had to rename robot %s to %s because dots not allowed in MORSE names"
                  % (rname, rnameSafe))
            for goalStr in GOALSTRINGS:
                goal = bpy.context.scene.objects.get(obj.name + goalStr)
                if goal:
                    print("\t> also renamed goal %s" % goal.name)
                    goal.name = rnameSafe + goalStr
            rname = rnameSafe
        # avoid name collision and mark for deletion
        obj.name += '_'
        to_delete.append(obj)
        # add the MORSE components
        robot = getattr(morse.builder, rtype)(rname)
        motion = getattr(morse.builder, ctype)(robot.name+'Motion')
        # copy pose
        robot.location = pos
        robot.rotation_euler = rot
        robot.append(motion)
        motion.add_service('socket')    # port = 4000
        i += 1
# delete the stand-in models
for obj in to_delete:
    bpy.context.scene.objects.unlink(obj)
# disallow sleeping
for obj in bpy.context.scene.objects:
    if obj.game.physics_type == 'RIGID_BODY':
            obj.game.use_sleep = True   # backwards; True means "no sleeping"

# Fix the timestep physics is advanced by each frame
#bpy.context.scene.game_settings.fps = 60
#bpy.context.scene.game_settings.use_frame_rate = True
#bpy.context.scene.game_settings.restrict_animation_updates = True
bpy.context.scene.game_settings.show_framerate_profile = True

# Get '__settings' object so we can set up some properties
settings = bpy.data.objects['__settings']
settings.hide = False
settings.hide_render = False
settings.hide_select = False

# Determine the solution path file to use (second argument)
outpath = sys.argv[sys.argv.index('--') + 2]
settings['Outpath'] = outpath

# Set the mode setting
settings['Mode'] = mode

# Mode specific configuration:
if mode == 'PLAY':
    bpy.context.scene.game_settings.use_animation_record = True
    
bpy.ops.object.select_all(action='DESELECT')
context_override = {"active_object":settings,"object":settings,"blend_data":bpy.data,
                    "scene":bpy.context.scene,"edit_object":settings}

# Add 'Tick' sensor
bpy.ops.logic.sensor_add(context_override, type='DELAY', name='Tick')
tick = settings.game.sensors['Tick']
tick.use_repeat = True

# Add 'communicator.py' text block
bpy.ops.text.open(filepath=OMPL_DIR + "/communicator.py")

# Add 'Comm' controller
bpy.ops.logic.controller_add(context_override, type='PYTHON', name='Comm')
comm = settings.game.controllers['Comm']
comm.mode = 'MODULE'
comm.module = 'communicator.main'

# Link Tick with Comm so it's run every frame
tick.link(comm)

env.create()



