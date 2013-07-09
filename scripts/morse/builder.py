# MORSE builder script
# requires name of the environment *.blend file as a parameter

import sys
import logging
import bpy
import morse.builder

# IMPORTANT! Set this manually for now
OMPL_DIR='/home/caleb/repos/ompl_morse'

# Disable logging of socket communication because there will be a lot of it
sockloggers = (logging.getLogger("morse.morse.core.request_manager"),
               logging.getLogger("morse.morse.middleware.socket_request_manager"))
sockloggers[0].setLevel(logging.ERROR)
sockloggers[1].setLevel(logging.ERROR)

# Add a robot (TODO: make user-specified)
robot = morse.builder.SegwayRMP400()
robot.unparent_wheels()
motion = morse.builder.MotionVWDiff()
robot.append(motion)
motion.add_service('socket')    # default port = 4000

# Determine the blend file to load (first argument after '--')
envpath = sys.argv[sys.argv.index('--') + 1]
print("\n* Loading scene <%s>.\n" % envpath)
env = morse.builder.Environment(envpath)

# Create '__planner' object so we can set up the game engine
bpy.ops.object.add()
obj = bpy.context.object
obj.name = "__planner"

# Add 'Tick' sensor
bpy.ops.logic.sensor_add(type='DELAY', name='Tick')
tick = obj.game.sensors['Tick']
tick.use_repeat = True

# Add 'communicator.py' text block
bpy.ops.text.open(filepath=OMPL_DIR + "scripts/morse/communicator.py")

# Add 'Comm' controller
bpy.ops.logic.controller_add(type='PYTHON', name='Comm')
comm = obj.game.controllers['Comm']
comm.mode = 'MODULE'
comm.module = 'communicator.main'

# Link Tick with Comm so it's run every frame
tick.link(comm)

# Double-check goal object properties
bpy.ops.object.make_single_user(type='ALL', material=True)
for obj in bpy.data.objects:
    if obj.name.endswith('.goal'):
        if obj.game.physics_type != 'STATIC':
            print("Warning: changing goal object %s to static." % obj.name)
            obj.game.physics_type = 'STATIC'
        if not obj.game.use_ghost:
            print("Warning: changing goal object %s to ghost." % obj.name)
            obj.game.use_ghost = True
        mat = obj.active_material
        mat.use_transparency = True
        mat.transparency_method = 'Z_TRANSPARENCY'
        mat.alpha = 0.25
        mat.use_cast_approximate = False
        mat.use_cast_buffer_shadows = False
        mat.use_shadows = False

env.create()



