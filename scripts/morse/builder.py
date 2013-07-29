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

# Load the .blend file (first argument after '--')
envpath = sys.argv[sys.argv.index('--') + 1]
print("\n* Loading scene <%s>.\n" % envpath)
env = morse.builder.Environment(envpath)

# Replace the robot(s)
to_delete = []
i = 0
for obj in bpy.context.selectable_objects:
    if obj.game.properties.get('RobotType'):
        rtype = obj.game.properties['RobotType'].value
        ctype = obj.game.properties['ControllerType'].value
        pos = obj.location
        rot = obj.rotation_euler
        # avoid name collision and mark for deletion
        obj.name += '_'
        to_delete.append(obj.name)
        # add the MORSE components
        robot = getattr(morse.builder, rtype)()
        robot.name = "robot_%i" % i # this is how it will be referenced over the socket
        motion = getattr(morse.builder, ctype)()
        motion.name = "motion_%i" % i  # referenced by robot_#.motion_#
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
bpy.ops.object.select_all(action='DESELECT')
for name in to_delete:
    bpy.ops.object.select_pattern(pattern=name, case_sensitive=True)
bpy.ops.object.delete()

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

# Determine the mode to use (third argument)
mode = sys.argv[sys.argv.index('--') + 3]
bpy.ops.object.game_property_new(type='STRING', name="Mode")
obj.game.properties['Mode'].value = mode

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



