import bpy
import os

def assign_material(name):
    ob = bpy.context.active_object

    # Get material
    mat = bpy.data.materials.get(name)

    # Assign it to object
    if ob.data.materials:
       # assign to 1st material slot
       ob.data.materials[0] = mat
    else:
       # no slots
        ob.data.materials.append(mat)
        
def delete_all():
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete(use_global=False)

    for item in bpy.data.meshes:
       bpy.data.meshes.remove(item)
       
def import_atlas():
    bpy.ops.import_mesh.ply(filepath="atlas.ply")
    assign_material('atlas')
    bpy.ops.object.modifier_add(type='SOLIDIFY')
    bpy.context.object.modifiers["Solidify"].thickness = 0.03
    bpy.context.object.modifiers["Solidify"].offset = 0.0
    
delete_all()
os.chdir('/home/zak/lab/code/ompl/build/')

#import_atlas()

bpy.ops.import_mesh.ply(filepath="graph.ply")
assign_material('graph')

bpy.ops.import_mesh.ply(filepath="path.ply")
assign_material('path')