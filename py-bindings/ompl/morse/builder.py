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

# MORSE builder script

import logging
import os
import subprocess
import sys

import bpy

import morse.builder
import morse.blender

OMPL_DIR = os.path.dirname(__file__)

print("OMPL builder script invocation: " + str(sys.argv))

# Determine the mode to use (third argument)
mode = sys.argv[sys.argv.index('--') + 3]

# Use wmctrl for window manipulation
# (fails silently if wmctrl not installed)
winID = subprocess.check_output(['bash', '-c', \
    'wmctrl -l | grep morse_default_autorun | awk \'{ print $1 }\'']).decode()[:-1]
# Set a meaningful title
if mode == 'PLAN':
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -T "OMPL MORSE Planner"' % winID])
elif mode == 'PLAY':
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -T "OMPL MORSE Player"' % winID])
elif mode == 'QUERY':
    subprocess.call(['bash', '-c', 'wmctrl -i -r %s -T "OMPL MORSE Control Query"' % winID])
# Hide the Blender window if we're planning or querying
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

# Replace the robot(s) stand-in models with actual MORSE robot objects
to_delete = []
i = 0
for obj in bpy.context.scene.objects:

    # In PLAY mode, delete the goals
    if [True for goalStr in ['.goalPose', '.goalRegion', '.goalRot'] if obj.name.endswith(goalStr)]:
        if mode == 'PLAY':
            to_delete.append(obj)
        continue

    # If this object is a robot
    if obj.get('RobotType'):

        rtype = obj['RobotType']
        ctype = obj['ControllerType']
        pos = obj.location
        rot = obj.rotation_euler

        # Make names acceptable for MORSE
        rname = obj.name
        rnameSafe = rname.replace('.', '_')
        if rname != rnameSafe:
            print("WARNING: had to rename robot %s to %s because dots not allowed in MORSE names"
                  % (rname, rnameSafe))
            for goalStr in ['.goalPose', '.goalRegion', '.goalRot']:
                goal = bpy.context.scene.objects.get(obj.name + goalStr)
                if goal:
                    print("\t> also renamed goal %s" % goal.name)
                    goal.name = rnameSafe + goalStr
            rname = rnameSafe

        # Avoid name collision and mark for deletion
        obj.name += '_'
        to_delete.append(obj)

        # Add the MORSE components
        robot = getattr(morse.builder, rtype)(rname)
        motion = getattr(morse.builder, ctype)(robot.name+'Motion')

        # Restore pose
        robot.location = pos
        robot.rotation_euler = rot
        robot.append(motion)
        motion.add_service('socket')
        i += 1

##
# \brief Recursively delete an object and its children
def _recurseDelete(obj):

    # Call this function on all the children
    for child in obj.children[:]:
        _recurseDelete(child)

    # Select only this object and delete it
    bpy.ops.object.select_pattern(pattern=obj.name, case_sensitive=True, extend=False)
    bpy.ops.object.delete()

# Delete the stand-in models
for obj in to_delete:
    _recurseDelete(obj)

# Disallow sleeping for rigid bodies
for obj in bpy.context.scene.objects:
    if obj.game.physics_type == 'RIGID_BODY':
        # True means "no sleeping"
        obj.game.use_sleep = True

settings = bpy.data.objects['ompl_settings']

# Determine the solution path file to use (second argument)
outpath = sys.argv[sys.argv.index('--') + 2]
settings['Outpath'] = outpath

# Set the mode setting
settings['Mode'] = mode

# Record animation data if we're doing playback
if mode == 'PLAY':
    bpy.context.scene.game_settings.use_animation_record = True

bpy.ops.object.select_all(action='DESELECT')

# Add 'Tick' sensor
bpy.ops.logic.sensor_add(type='DELAY', name='Tick', object='ompl_settings')
tick = settings.game.sensors['Tick']
tick.use_repeat = True

# Add 'communicator.py' text block
bpy.ops.text.open(filepath=OMPL_DIR + "/communicator.py")

# Add 'Comm' controller for the script
bpy.ops.logic.controller_add(type='PYTHON', name='Comm', object='ompl_settings')
comm = settings.game.controllers['Comm']
comm.mode = 'MODULE'
comm.module = 'communicator.main'

# Link Tick with Comm so it's run every frame
tick.link(comm)

# Create the environment
env.create()
