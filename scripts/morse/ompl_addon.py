# ompl_addon.py
# To be installed as a Blender addon

bl_info = {
    "name":"OMPL Interface",
    "category":"Game Engine",
    "description":"Plan planning with OMPL (requires MORSE)",
    "location":"Game > OMPL",
    "author":"Caleb Voss"
}

# IMPORTANT! Set this manually for now
OMPL_DIR='/home/caleb/repos/ompl_morse'

import subprocess
import configparser
import os

import bpy

# Addon operator

class Plan(bpy.types.Operator):
    """Invoke OMPL Planning"""
    bl_idname = "ompl.plan"
    bl_label = "Plan..."
    
    def execute(self, context):
        """
        Called when this operator is run.
        """
        
        print('Starting planner...')
        print(bpy.data.filepath)
        #old MORSE interface: subprocess.call(['morse', '-c', 'run', 'ompl', 'builder.py', '--', bpy.data.filepath])
        subprocess.call(['morse', '-c', 'run', 'builder.py', '--', bpy.data.filepath])
        
        return {'FINISHED'}


# Menus

class OMPLMenu(bpy.types.Menu):
    bl_idname = "INFO_MT_game_ompl"
    bl_label = "OMPL"
    
    def draw(self, context):
        self.layout.operator(Plan.bl_idname)


def menu_func(self, context):
    self.layout.menu(OMPLMenu.bl_idname)

# Addon enable/disable functions

def register():
    """
    Called when this addon is enabled or Blender starts.
    """
    
    # uncomment this for old MORSE interface
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
    bpy.utils.register_class(OMPLMenu)
    bpy.types.INFO_MT_game.prepend(menu_func)

def unregister():
    """
    Called when this addon is disabled.
    """
    bpy.utils.unregister_class(Plan)
    bpy.utils.unregister_class(OMPLMenu)
    bpy.types.INFO_MT_game.remove(menu_func)




