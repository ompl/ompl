# ompl_addon.py
# To be installed as a Blender addon

bl_info = {
    "name":"OMPL Interface",
    "category":"Game Engine",
    "description":"Planning with OMPL (requires MORSE)",
    "location":"Game > OMPL",
    "author":"Caleb Voss"
}

# IMPORTANT! Set this manually for now
OMPL_DIR='/home/caleb/repos/ompl_morse'

import subprocess
import configparser
import os

import bpy
import mathutils

import sys
# the following path is in sys.path when you start python3 normally, but not when in Blender?!?
sys.path.append('/usr/local/lib/python3.2/dist-packages')
import morse.builder

# Addon operators

class Plan(bpy.types.Operator):
    """Invoke OMPL Planning"""
    bl_idname = "ompl.plan"
    bl_label = "Plan..."
    
    # automatically set by the Blender file selector dialog
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")
    
    def execute(self, context):
        """
        Called when this operator is run.
        """
        
        print('Starting planner...')
        print("Planning on %s, saving to %s" % (bpy.data.filepath, self.filepath))
        subprocess.Popen(['morse', '-c', 'run', OMPL_DIR+'/scripts/morse/builder.py', '--', bpy.data.filepath, self.filepath, 'PLAN'])
        # for the newer MORSE interface, use this instead of the above line:
        #subprocess.Popen(['morse', '-c', 'run', 'ompl', 'OMPL_DIR+'/scripts/morse/builder.py', '--', bpy.data.filepath, self.filepath, 'PLAN'])
        
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

class Play(bpy.types.Operator):
    """Invoke OMPL Playback"""
    bl_idname = "ompl.play"
    bl_label = "Play..."
    
    # automatically set by the Blender file selector dialog
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")
    
    def execute(self, context):
        """
        Called when this operator is run.
        """
        
        print('Starting player...')
        print("Playing %s with %s" % (bpy.data.filepath, self.filepath))
        subprocess.Popen(['morse', '-c', 'run', OMPL_DIR+'/scripts/morse/builder.py', '--', bpy.data.filepath, self.filepath, 'PLAY'])
        # for the newer MORSE interface, use this instead of the above line:
        #subprocess.Popen(['morse', '-c', 'run', 'ompl', 'OMPL_DIR+'/scripts/morse/builder.py', '--', bpy.data.filepath, self.filepath, 'PLAY'])
        
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

# helpers for AddRobot class

def getRobots():
    """
    Compile list of valid MORSE robots.
    """
    excluded_robots = []    #TODO
    robotEnum = []
    i=0
    for cname in dir(morse.builder.robots):
        c = getattr(morse.builder.robots, cname)
        # is c a class?
        if isinstance(c, type):
            # does it inherit from Robot and is it neither Robot nor WheeledRobot?
            if issubclass(c, morse.builder.Robot) and c != morse.builder.Robot and c != morse.builder.WheeledRobot:
                # is is not in our exlusions list?
                if cname not in excluded_robots:
                    robotEnum.append((cname,cname,'morse.builder.robots.'+cname,i))
                    i += 1
    robotEnum.reverse()
    return robotEnum

def getControllers():
    """
    Compile list of controllers valid for the selected robot.
    """
    # exclude controllers that have non-numeric parameters, don't have a socket interface, or are irrelevant
    excluded_controllers = ['Armature','Destination','ForceTorque','Gripper','Keyboard','KukaLWR',
        'Light','Mocap','MocapControl','MotionXYW','Orientation','PTU','RotorcraftAttitude','SteerForce']
    controllerEnum = []
    i=0
    for cname in dir(morse.builder.actuators):
        c = getattr(morse.builder.actuators, cname)
        # is c a class?
        if isinstance(c, type):
            # does it inherit from Actuator and is it not Actuator?
            # OR does it inherit from ActuatorCreator and is it not ActuatorCreator?
            if (issubclass(c, morse.builder.Actuator) and
                c != morse.builder.Actuator) or \
                (issubclass(c, morse.builder.creator.ActuatorCreator) and
                c != morse.builder.creator.ActuatorCreator):
                # is it not in our exclusions list?
                if cname not in excluded_controllers:
                    controllerEnum.append((cname,cname,'morse.builder.actuators.'+cname,i))
                    i += 1
    controllerEnum.reverse()
    return controllerEnum
        
class AddRobot(bpy.types.Operator):
    """Add a MORSE Robot to the Scene"""
    bl_idname = "ompl.addrobot"
    bl_label = "Add Robot"
    
    # automatically set by the Blender properties dialog
    robotEnum = getRobots()
    controllerEnum = getControllers()
    robot_type = bpy.props.EnumProperty(items=robotEnum, name="MORSE robot", default=robotEnum[-1][0])
    controller_type = bpy.props.EnumProperty(items=controllerEnum,
        name="MORSE actuator", default=controllerEnum[-1][0])
    
    def execute(self, context):
        """
        Called when this operator is run.
        """
        
        # add model for robot_type
        robot = getattr(morse.builder, self.robot_type)()
        robotObj = bpy.context.object
        
        # join all it's children
        bpy.ops.object.select_all(action='DESELECT')
        for child in robotObj.children:
            bpy.ops.object.select_pattern(pattern=child.name, case_sensitive=True)
        bpy.ops.object.select_pattern(pattern=robotObj.name, case_sensitive=True)
        bpy.ops.object.join()
        
        # remove unnecessary game properties
        while len(robotObj.game.properties) > 0:
            bpy.ops.object.game_property_remove()
        
        # add game properties for robot_type and controller_type
        bpy.ops.object.game_property_new(type='STRING', name="RobotType")
        bpy.context.object.game.properties['RobotType'].value = self.robot_type
        bpy.ops.object.game_property_new(type='STRING', name="ControllerType")
        bpy.context.object.game.properties['ControllerType'].value = self.controller_type
        
        
        return {'FINISHED'}

    def invoke(self, context, event):
        """
        Called when the button is pressed.
        """
        
        # choose robot and controller
        return bpy.context.window_manager.invoke_props_dialog(self)

# Menus

class OMPLMenu(bpy.types.Menu):
    bl_idname = "INFO_MT_game_ompl"
    bl_label = "OMPL"
    
    def draw(self, context):
        self.layout.operator_context = 'INVOKE_DEFAULT'
        self.layout.operator(Plan.bl_idname)
        self.layout.operator(Play.bl_idname)
        self.layout.operator(AddRobot.bl_idname)


def menu_func(self, context):
    self.layout.menu(OMPLMenu.bl_idname)

# Addon enable/disable functions

def register():
    """
    Called when this addon is enabled or Blender starts.
    """
    
    # uncomment for the latest MORSE interface
    """
    # Ensure that MORSE environment 'ompl' is registered in ~/.morse/config
    config_path = os.path.expanduser("~/.morse")
    if not os.path.exists(config_path):
        os.mkdir(config_path)
    config_file = os.path.join(config_path, "config")

    conf = configparser.SafeConfigParser()
    conf.read(config_file)
    if not conf.has_section("sites"):
        conf.add_section("sites")
    conf.set('sites', 'ompl', OMPL_DIR + '/scripts/morse')

    with open(config_file, 'w') as configfile:
        conf.write(configfile)
    """
    
    # Set up menus
    bpy.utils.register_class(Plan)
    bpy.utils.register_class(Play)
    bpy.utils.register_class(AddRobot)
    bpy.utils.register_class(OMPLMenu)
    bpy.types.INFO_MT_game.prepend(menu_func)

def unregister():
    """
    Called when this addon is disabled.
    """
    bpy.utils.unregister_class(Plan)
    bpy.utils.unregister_class(Play)
    bpy.utils.unregister_class(AddRobot)
    bpy.utils.unregister_class(OMPLMenu)
    bpy.types.INFO_MT_game.remove(menu_func)




