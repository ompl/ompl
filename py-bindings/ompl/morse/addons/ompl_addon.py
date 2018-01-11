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

## \brief Information about this addon
bl_info = {
    "name":"OMPL Interface",
    "category":"Game Engine",
    "description":"Planning with OMPL (requires MORSE)",
    "location":"Game > OMPL",
    "author":"Caleb Voss"
}

import configparser
import os
import socket
import subprocess
import sys
import time

import bpy

import ompl.morse.environment

OMPL_DIR = ompl.morse.__path__[0]

inf = float('inf')

# #
# Addon operators (actions in the menu the user can execute)
# #

##
# \brief Invoke OMPL Planning
class Plan(bpy.types.Operator):

    bl_idname = "ompl.plan"
    bl_label = "Plan..."

    ## \brief File where planner should save solution path
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    ##
    # \brief Called when the dialogs finish; starts up the simulation
    def execute(self, _):

        print('Starting planner...')
        print("Planning on %s, saving to %s" % (bpy.data.filepath, self.filepath))
        subprocess.Popen(['morse', '-c', 'run', 'ompl', OMPL_DIR+'/builder.py', '--', \
            bpy.data.filepath, self.filepath, 'PLAN'])

        return {'FINISHED'}

    ##
    # \brief Called when the button is pressed; double-check configuration and
    #    ask for a file to save the path to
    def invoke(self, context, _):

        # Double-check goal object properties to make sure they're out of the way and
        #   connected properly
        for obj in bpy.data.objects:
            if [True for goalStr in ['.goalPose', '.goalRegion', '.goalRot'] \
                if obj.name.endswith(goalStr)]:
                obj.hide_render = True
                if obj.name.endswith('.goalRegion'):
                    obj.game.physics_type = 'SENSOR'
                    body = bpy.data.objects.get(obj.name[:-11])
                    if not body:
                        continue
                    collider = obj.game.sensors.get("__collision")
                    if not collider:
                        # Link up a collision sensor
                        bpy.ops.logic.sensor_add(type='COLLISION', name="__collision", object=obj.name)
                        collider = obj.game.sensors.get("__collision")
                    collider.property = body.name.replace('.', '_')
                    # Just to make the sensor active
                    dummy = obj.game.controllers.get("__dummy")
                    if not dummy:
                        bpy.ops.logic.controller_add(type='EXPRESSION', name="__dummy", object=obj.name)
                        dummy = obj.game.controllers["__dummy"]
                    dummy.expression = 'TRUE'
                    collider.link(dummy)
                else:
                    obj.game.physics_type = 'NO_COLLISION'

        if not context.scene.objects.get('ompl_settings'):
            # Bounds Configuration hasn't been setup for this file yet
            bpy.ops.ompl.boundsconfig('INVOKE_DEFAULT')
        else:
            settings = context.scene.objects['ompl_settings']
            if settings.get('autopb') is None:
                # Bounds Configuration hasn't been setup for this file yet
                bpy.ops.ompl.boundsconfig('INVOKE_DEFAULT')

        # Save any changes so MORSE sees them when it loads the file
        bpy.ops.wm.save_mainfile()

        # File selector for the path output file
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

@bpy.app.handlers.persistent
def import_and_resave(animpath):
    bpy.app.handlers.load_post.clear()
    animtmppath = animpath + ".tmp"
    print("OMPL: appending animation data")
    with bpy.data.libraries.load(filepath=animtmppath) as (_, t):
        t.scenes = ['S.MORSE_LOGIC']
    print("OMPL: deleting tmp file")
    os.remove(animtmppath)
    bpy.data.scenes.remove(bpy.data.scenes['Scene'])
    bpy.data.scenes['S.MORSE_LOGIC'].name = 'Scene'
    bpy.context.screen.scene = bpy.data.scenes['Scene']
    bpy.ops.wm.save_mainfile(filepath=animpath)

##
# \brief Invoke Path Playback
class Play(bpy.types.Operator):

    bl_idname = "ompl.play"
    bl_label = "Playback and save"

    ## \brief File where the planner wrote the solution path
    filepath = bpy.props.StringProperty(name="Solution file", \
        description="File where where the OMPL planner saved a solution path", subtype="FILE_PATH")

    ##
    # \brief Called when the dialogs finish; starts up the simulation
    def execute(self, context):

        animpath = context.scene.objects['ompl_settings']['Animpath']
        if animpath == '':
            self.report({'ERROR'}, "Choose animation save file first!")
            return {'FINISHED'}
        self.report({'WARNING'}, "Switching to .blend file: '" + animpath + "'")

        print('Starting player...')
        print("Playing %s with %s" % (bpy.data.filepath, self.filepath))
        subprocess.run(['morse', '-c', 'run', 'ompl', OMPL_DIR+'/builder.py', '--', bpy.data.filepath, self.filepath, 'PLAY'])

        # Load blank file. Append animated objects. Re-save.
        print("OMPL: Will save animation data to '" + animpath + "'")
        cont = bpy.app.handlers.persistent(lambda _: import_and_resave(animpath))
        bpy.app.handlers.load_post.append(cont)
        blankpath = OMPL_DIR + '/resources/blank.blend'
        print("OMPL: Loading blank file")
        bpy.ops.wm.open_mainfile(filepath=blankpath)

        return {'FINISHED'}

    ##
    # \brief Called when the button is pressed; prompts for the path file
    def invoke(self, context, _):

        if not context.scene.objects.get('ompl_settings'):
            # Select an animation save file
            bpy.ops.ompl.animfile('INVOKE_DEFAULT')
        elif not context.scene.objects['ompl_settings'].get('Animpath'):
            # Select an animation save file
            bpy.ops.ompl.animfile('INVOKE_DEFAULT')

        # Save any changes so MORSE sees them when it loads the file
        bpy.ops.wm.save_mainfile()

        # File selector for the path file
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

##
# \brief Compile a list of usable MORSE robots
def getRobots():

    import morse.builder

    # This is a list of incompatible robots (e.g., some use controllers that require you to explicitly
    #   name the internal variable you want to change instead of merely accepting a list of control values).
    #   If you write your own controller that is compatible, feel free to take the robot out of this blacklist
    excluded_robots = ['B21', 'BarePR2', 'BasePR2', 'Human', 'Hummer', 'Jido', \
        'LocalizedPR2', 'NavPR2', 'Victim']
    robotEnum = []
    i = 0
    for cname in dir(morse.builder.robots):
        c = getattr(morse.builder.robots, cname)
        # Is c a class?
        if isinstance(c, type):
            # Does it inherit from Robot and is it neither Robot nor WheeledRobot?
            if issubclass(c, morse.builder.Robot) and c != morse.builder.Robot and c != morse.builder.WheeledRobot:
                # Is is not in our exlusions list?
                if cname not in excluded_robots:
                    # Add an entry for it
                    robotEnum.append((cname, cname, 'morse.builder.robots.' + cname, i))
                    i += 1
    # Put then in alphabetical order
    robotEnum.reverse()
    return robotEnum

##
# \brief Compile list of controllers
def getControllers():

    import morse.builder

    # Exclude controllers that require non-numeric parameters, don't have a socket interface, or are irrelevant;
    #   you may be able to rewrite some of these (e.g., SteerForce) with little modification so that they do
    #   accept purely numeric inputs
    excluded_controllers = ['Armature', 'Destination', 'ForceTorque', 'Gripper', 'Joystick', \
        'Keyboard', 'KukaLWR', 'Light', 'Mocap', 'MocapControl', 'MotionXYW', 'Orientation', \
        'PTU', 'RotorcraftAttitude', 'Sound', 'SteerForce']
    controllerEnum = []
    i = 0
    for cname in dir(morse.builder.actuators):
        c = getattr(morse.builder.actuators, cname)
        # Is c a class?
        if isinstance(c, type):
            # Does it inherit from ActuatorCreator and is it not ActuatorCreator?
            if issubclass(c, morse.builder.creator.ActuatorCreator) and \
                c != morse.builder.creator.ActuatorCreator:
                # Is it not in our exclusions list?
                if cname not in excluded_controllers:
                    # Add an entry for it
                    controllerEnum.append((cname, cname, 'morse.builder.actuators.' + cname, i))
                    i += 1
    controllerEnum.reverse()
    return controllerEnum

##
# \brief Add a MORSE Robot to the scene
class AddRobot(bpy.types.Operator):

    bl_idname = "ompl.addrobot"
    bl_label = "Add Robot..."

    # Set up the robot and controller selection menus
    robotEnum = [('', '', '')]
    controllerEnum = [('', '', '')]

    robot_type = bpy.props.EnumProperty(items=robotEnum, name="MORSE robot", \
        description="A robot from the MORSE components library", default=robotEnum[-1][0])
    controller_type = bpy.props.EnumProperty(items=controllerEnum, name="MORSE actuator", \
        description="The actuator to control the robot", default=controllerEnum[-1][0])


    ##
    # \brief Operator refuses to run if this returns false; requires
    #    Blender to be in Object Mode
    @classmethod
    def poll(cls, context):

        return context.mode == 'OBJECT'

    ##
    # \brief Add the model to the scene and set up some properties
    def execute(self, context):

        import morse.builder

        # Add model for robot_type
        robot = getattr(morse.builder, self.robot_type)()
        robotObj = context.object

        # Make visible in a render
        robotObj.hide_render = False

        # Remove unnecessary game properties
        while robotObj.game.properties:
            bpy.ops.object.game_property_remove()

        # Add properties for robot_type and controller_type
        robotObj['RobotType'] = self.robot_type
        robotObj['ControllerType'] = self.controller_type

        return {'FINISHED'}

    ##
    # \brief Prompt for robot and controller selection
    def invoke(self, context, _):
        return context.window_manager.invoke_props_dialog(self)

##
# \brief Recursively add children to the selection
def _recurseSelectChildren(obj):

    for child in obj.children:
        _recurseSelectChildren(child)

    bpy.ops.object.select_pattern(pattern=obj.name, case_sensitive=True)


##
# \brief Add a goal to the Scene
class AddGoal(bpy.types.Operator):

    bl_idname = "ompl.addgoal"
    bl_label = "Add Goal..."

    # Parameters are the type of goal and the name of the object we define the goal for
    body = bpy.props.StringProperty(name="Rigid Body", \
        description="The body to define a goal for", default="")
    goal_type = bpy.props.EnumProperty(items=[('goalRot', 'Rotation only', 'Rotation'), \
        ('goalPose', 'Pose', 'Position and Rotation')], name="Goal Type", \
        description="The kind of goal specification", default='goalPose')

    ##
    # \brief Operator refuses to run if this returns false; requires
    #    Blender to be in Object mode
    @classmethod
    def poll(cls, context):

        return context.mode == 'OBJECT'

    ##
    # \brief Create the goal object and set up its properties
    def execute(self, context):

        # Check that the object exists
        if not bpy.data.objects.get(self.body):
            self.report({'ERROR'}, "No such object: '%s'" % self.body)
            return {'FINISHED'}

        goalname = self.body + '.' + self.goal_type
        bpy.ops.object.select_all(action='DESELECT')

        # Duplicate object
        bpy.ops.object.select_pattern(pattern=self.body, case_sensitive=True)
        _recurseSelectChildren(bpy.data.objects.get(self.body))
        bpy.ops.object.duplicate()
        goalobj = context.selected_objects[0]

        # Remove old custom properties
        for prop in goalobj.keys():
            del goalobj[prop]

        if self.goal_type == 'goalPose':
            # Add default locTol
            goalobj['locTol'] = 0.5

        # Add default rotTol
        goalobj['rotTol'] = 0.2

        # Rename goal object
        goalobj.name = goalname

        # Move object to cursor
        goalobj.location = context.scene.cursor_location

        return {'FINISHED'}

    ##
    # \brief Prompt for the object name and goal type
    def invoke(self, context, _):
        return context.window_manager.invoke_props_dialog(self)


##
# \brief Choose animation save file
class AnimFile(bpy.types.Operator):

    bl_idname = "ompl.animfile"
    bl_label = "Choose animation save file..."

    ## \brief Second *.blend to save the animation data
    filepath = bpy.props.StringProperty(name="Animation Save file", \
        description="*.blend file where the animation curves should be saved to", subtype="FILE_PATH")

    ##
    # \brief Save the name of the file for later
    def execute(self, context):

        context.scene.objects['ompl_settings']['Animpath'] = self.filepath
        return {'FINISHED'}

    ##
    # \brief Prompt for the animation save file
    def invoke(self, context, _):
        # Add the settings object if it doesn't exist
        if not context.scene.objects.get('ompl_settings'):
            bpy.ops.object.add()
            context.object.name = 'ompl_settings'
        settings = context.scene.objects['ompl_settings']

        # Get the settings object out of the way
        settings.hide = True
        settings.hide_render = True
        settings.hide_select = True

        if not settings.get('Animpath'):
            settings['Animpath'] = self.filepath

        # Prompt for the name of the file to save to
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


##
# \brief Configure the state and control bounds
class BoundsConfiguration(bpy.types.Operator):

    bl_idname = "ompl.boundsconfig"
    bl_label = "Bounds Configuration..."

    # Properties displayed in the dialog; p=position,l=linear,a=angular,c=control;
    #   x,y,z,m=min, X,Y,Z,M=max; handles up to 16 control inputs
    autopb = bpy.props.BoolProperty(name="Automatic position bounds", \
        description="Overrides user-provided numbers by analyzing the scene", \
        default=True)
    pbx = bpy.props.FloatProperty(name="Min", default=-1000, min=-1000, max=1000)
    pbX = bpy.props.FloatProperty(name="Max", default=1000, min=-1000, max=1000)
    pby = bpy.props.FloatProperty(name="Min", default=-1000, min=-1000, max=1000)
    pbY = bpy.props.FloatProperty(name="Max", default=1000, min=-1000, max=1000)
    pbz = bpy.props.FloatProperty(name="Min", default=-1000, min=-1000, max=1000)
    pbZ = bpy.props.FloatProperty(name="Max", default=1000, min=-1000, max=1000)
    lbm = bpy.props.FloatProperty(name="Min", default=-1000, min=-1000, max=1000)
    lbM = bpy.props.FloatProperty(name="Max", default=1000, min=-1000, max=1000)
    abm = bpy.props.FloatProperty(name="Min", default=-1000, min=-1000, max=1000)
    abM = bpy.props.FloatProperty(name="Max", default=1000, min=-1000, max=1000)
    cbm0 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM0 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm1 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM1 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm2 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM2 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm3 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM3 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm4 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM4 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm5 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM5 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm6 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM6 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm7 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM7 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm8 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM8 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm9 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM9 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm10 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM10 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm11 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM11 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm12 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM12 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm13 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM13 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm14 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM14 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)
    cbm15 = bpy.props.FloatProperty(name="Min", default=-10, min=-1000, max=1000)
    cbM15 = bpy.props.FloatProperty(name="Max", default=10, min=-1000, max=1000)

    ##
    # \brief Save all the settings and reset dialogs to new defaults
    def execute(self, context):

        # Save settings to the scene
        settings = context.scene.objects['ompl_settings']
        settings['autopb'] = self.autopb
        settings['pbx'] = self.pbx
        settings['pbX'] = self.pbX
        settings['pby'] = self.pby
        settings['pbY'] = self.pbY
        settings['pbz'] = self.pbz
        settings['pbZ'] = self.pbZ
        settings['lbm'] = self.lbm
        settings['lbM'] = self.lbM
        settings['abm'] = self.abm
        settings['abM'] = self.abM
        for i in range(16):
            settings['cbm%i'%i] = getattr(self, 'cbm%i'%i)
            settings['cbM%i'%i] = getattr(self, 'cbM%i'%i)

        # Allow dialog defaults to be changed by resetting the properties
        del BoundsConfiguration.autopb, BoundsConfiguration.pbx, BoundsConfiguration.pbX,\
            BoundsConfiguration.pby, BoundsConfiguration.pbY, BoundsConfiguration.pbz,\
            BoundsConfiguration.pbZ, BoundsConfiguration.lbm, BoundsConfiguration.lbM,\
            BoundsConfiguration.abm, BoundsConfiguration.abM
        for i in range(16):
            delattr(BoundsConfiguration, 'cbm%i'%i)
            delattr(BoundsConfiguration, 'cbM%i'%i)

        BoundsConfiguration.autopb = bpy.props.BoolProperty(name="Automatic position bounds", \
            description="Overrides user-provided numbers by analyzing the scene", \
            default=settings['autopb'])
        BoundsConfiguration.pbx = bpy.props.FloatProperty(name="Min", default=settings['pbx'], min=-1000, max=1000)
        BoundsConfiguration.pbX = bpy.props.FloatProperty(name="Max", default=settings['pbX'], min=-1000, max=1000)
        BoundsConfiguration.pby = bpy.props.FloatProperty(name="Min", default=settings['pby'], min=-1000, max=1000)
        BoundsConfiguration.pbY = bpy.props.FloatProperty(name="Max", default=settings['pbY'], min=-1000, max=1000)
        BoundsConfiguration.pbz = bpy.props.FloatProperty(name="Min", default=settings['pbz'], min=-1000, max=1000)
        BoundsConfiguration.pbZ = bpy.props.FloatProperty(name="Max", default=settings['pbZ'], min=-1000, max=1000)
        BoundsConfiguration.lbm = bpy.props.FloatProperty(name="Min", default=settings['lbm'], min=-1000, max=1000)
        BoundsConfiguration.lbM = bpy.props.FloatProperty(name="Max", default=settings['lbM'], min=-1000, max=1000)
        BoundsConfiguration.abm = bpy.props.FloatProperty(name="Min", default=settings['abm'], min=-1000, max=1000)
        BoundsConfiguration.abM = bpy.props.FloatProperty(name="Max", default=settings['abM'], min=-1000, max=1000)
        for i in range(16):
            setattr(BoundsConfiguration, 'cbm%i'%i, bpy.props.FloatProperty(name="Min", default=settings['cbm%i'%i], min=-1000, max=1000))
            setattr(BoundsConfiguration, 'cbM%i'%i, bpy.props.FloatProperty(name="Max", default=settings['cbM%i'%i], min=-1000, max=1000))

        # Refresh
        bpy.utils.unregister_class(BoundsConfiguration)
        bpy.utils.register_class(BoundsConfiguration)

        return {'FINISHED'}

    ##
    # \brief Query MORSE for control description, then open the dialog
    def invoke(self, context, _):
        # If the settings have not been set before, initialize them
        if not context.scene.objects.get('ompl_settings'):
            bpy.ops.object.add()
            settings = context.object
            settings.name = 'ompl_settings'
            settings['autopb'] = True
            settings['pbx'] = -1000
            settings['pbX'] = 1000
            settings['pby'] = -1000
            settings['pbY'] = 1000
            settings['pbz'] = -1000
            settings['pbZ'] = 1000
            settings['lbm'] = -1000
            settings['lbM'] = 1000
            settings['abm'] = -1000
            settings['abM'] = 1000
            for i in range(16):
                settings['cbm%i'%i] = -10
                settings['cbM%i'%i] = 10

        # Save any changes so MORSE sees them when it loads the file
        bpy.ops.wm.save_mainfile()

        # Query MORSE for cdesc by starting it up temporarily (clunky, but it needs to be done)
        subprocess.Popen(['morse', '-c', 'run', 'ompl', OMPL_DIR+'/builder.py', '--', bpy.data.filepath, ".", 'QUERY'])

        # Wait for a connection
        sockS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sockC = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:
            try:
                print("Waiting for port 50007 to connect.")
                sockS.connect(('localhost', 50007))
            except:
                time.sleep(0.5)
                continue
            break
        while True:
            try:
                print("Waiting for port 4000 to connect.")
                sockC.connect(('localhost', 4000))
            except:
                time.sleep(0.5)
                continue
            break

        # Retrieve the control description
        self.cdesc = ompl.morse.environment.MyEnvironment(sockS, sockC, True).cdesc
        if self.cdesc[0] > 16:
            self.report({'ERROR'}, "OMPL Error: Control dimension exceeds 16! This dialog won't be able to accomdate that many.")
            return {'FINISHED'}

        # Invoke bounds dialog
        return context.window_manager.invoke_props_dialog(self, width=1100)

    ##
    # \brief
    def draw(self, _):
        mainlayout = self.layout.row()
        # 3 sections in first column:
        sections = mainlayout.column()
        sections.label(text="Position Bounds:")
        sections.prop(self, 'autopb')
        pb = sections.row()
        sections.separator()
        sections.label(text="Linear Velocity Bounds:")
        lb = sections.row()
        sections.separator()
        sections.label(text="Angular Velocity Bounds:")
        ab = sections.row()
        # 1 section in second column
        cb = mainlayout.column()
        cb.label(text="Control Input Bounds:")
        cbrow1 = cb.row()
        cbrow2 = cb.row()
        cbrow3 = cb.row()
        cbrow4 = cb.row()

        # In positional bounds sections, make 3 columns for X,Y,Z, with Min,Max in each
        X = pb.column()
        X.label(text="X")
        X.prop(self, 'pbx', text="Min")
        X.prop(self, 'pbX', text="Max")
        Y = pb.column()
        Y.label(text="Y")
        Y.prop(self, 'pby', text="Min")
        Y.prop(self, 'pbY', text="Max")
        Z = pb.column()
        Z.label(text="Z")
        Z.prop(self, 'pbz', text="Min")
        Z.prop(self, 'pbZ', text="Max")

        # Linear velocity bounds Min,Max
        lb.prop(self, 'lbm', text="Min")
        lb.prop(self, 'lbM', text="Max")

        # Angular
        ab.prop(self, 'abm', text="Min")
        ab.prop(self, 'abM', text="Max")

        # Control Input
        last_component = None
        i = 0
        k = 0
        cbrow = [cbrow1, cbrow2, cbrow3, cbrow4]
        for control in self.cdesc[2:]:
            if control[0] != last_component:
                # Only allow 4 robots per row
                robot = cbrow[int(k/4)].column()
                k += 1
                # Print the robot name
                robot.label(text=control[0][:-6]+":")
                services = robot.box()
            # Print the controller function name
            services.label(text=control[1]+":")
            args = services.row()
            for j in range(control[2]):
                # Print the argument number
                con = args.column()
                con.label(text="Arg %i"%j)
                con.prop(self, 'cbm%i'%i, text="Min")
                con.prop(self, 'cbM%i'%i, text="Max")
                i += 1

# #
# Addon house-keeping
# #

##
# \brief Class describing the layout of the OMPL menu
class OMPLMenu(bpy.types.Menu):

    bl_idname = "INFO_MT_game_ompl"
    bl_label = "OMPL"

    ##
    # \brief Add operators to the menu
    def draw(self, _):
        self.layout.operator_context = 'INVOKE_DEFAULT'
        self.layout.operator(AddRobot.bl_idname)
        self.layout.operator(AddGoal.bl_idname)
        self.layout.operator(BoundsConfiguration.bl_idname)
        self.layout.operator(Plan.bl_idname)
        self.layout.operator(AnimFile.bl_idname)
        self.layout.operator(Play.bl_idname)

##
# \brief Function called to initialize the menu
def menu_func(self, _):
    self.layout.menu(OMPLMenu.bl_idname)

##
# \brief Deferred import of morse.builder (whenever a new file is loaded)
@bpy.app.handlers.persistent
def handler_scene_update_post(_):
    # A little hackish, but now is a good time to import morse.builder
    if 'morse.builder' not in sys.modules:
        del AddRobot.robot_type
        del AddRobot.controller_type
        robotEnum = getRobots()
        controllerEnum = getControllers()
        AddRobot.robot_type = bpy.props.EnumProperty(items=robotEnum, name="MORSE robot", \
            description="A robot from the MORSE components library", default=robotEnum[-1][0])
        AddRobot.controller_type = bpy.props.EnumProperty(items=controllerEnum, name="MORSE actuator", \
            description="The actuator to control the robot", default=controllerEnum[-1][0])
        bpy.utils.unregister_class(AddRobot)
        bpy.utils.register_class(AddRobot)


##
# \brief Called when the addon is enabled or Blender starts
def register():

    # Ensure that MORSE environment 'ompl' is registered in ~/.morse/config
    config_path = os.path.expanduser("~/.morse")
    if not os.path.exists(config_path):
        os.mkdir(config_path)
    config_file = os.path.join(config_path, "config")

    conf = configparser.SafeConfigParser()
    conf.read(config_file)
    if not conf.has_section("sites"):
        conf.add_section("sites")
    conf.set('sites', 'ompl', OMPL_DIR)

    with open(config_file, 'w') as configfile:
        conf.write(configfile)

    # Register all the operators, menu, and handler
    bpy.utils.register_class(Plan)
    bpy.utils.register_class(AnimFile)
    bpy.utils.register_class(Play)
    bpy.utils.register_class(AddRobot)
    bpy.utils.register_class(AddGoal)
    bpy.utils.register_class(BoundsConfiguration)
    bpy.utils.register_class(OMPLMenu)
    bpy.types.INFO_MT_game.prepend(menu_func)
    bpy.app.handlers.scene_update_post.append(handler_scene_update_post)

##
# \brief Called when operator is uninstalled
def unregister():

    # Undo all the registering
    bpy.utils.unregister_class(Plan)
    bpy.utils.unregister_class(AnimFile)
    bpy.utils.unregister_class(Play)
    bpy.utils.unregister_class(AddRobot)
    bpy.utils.unregister_class(AddGoal)
    bpy.utils.unregister_class(BoundsConfiguration)
    bpy.utils.unregister_class(OMPLMenu)
    bpy.types.INFO_MT_game.remove(menu_func)
    bpy.app.handlers.scene_update_post.remove(handler_scene_update_post)
