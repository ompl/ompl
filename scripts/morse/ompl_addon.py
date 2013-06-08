# ompl_addon.py
# To be installed as a Blender addon

bl_info = {
    "name":"OMPL Interface",
    "category":"Game Engine",
    "description":"Plan planning with OMPL (requires MORSE)",
    "location":"Game > OMPL",
    "author":"Caleb Voss"
}

import bpy

class Plan(bpy.types.Operator):
    """Invoke OMPL Planning"""
    bl_idname = "ompl.plan"
    bl_label = "Plan..."
    
    def execute(self, context):
        """
        Called run this operator is run.
        """
        
        print('Doing stuff')
        
        return {'FINISHED'}


class OMPLMenu(bpy.types.Menu):
    bl_idname = "INFO_MT_game_ompl"
    bl_label = "OMPL"
    
    def draw(self, context):
        self.layout.operator(Plan.bl_idname)


def menu_func(self, context):
    self.layout.menu(OMPLMenu.bl_idname)


def register():
    """
    Called when this addon is enabled.
    """
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




