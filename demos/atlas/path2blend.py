import bpy
import os

os.chdir('/home/zak/lab/code/ompl/build/')

# Syntax for each mapping entry is ("object name", "property name", [list of values])
#  where a value of None indicates no change is to be made.
_mapping5 = lambda s: [("Node1", 'location', s[0:3]), ("Node2", 'location', s[3:6]), ("Node3", 'location', s[6:9]),
                     ("Node4", 'location', s[9:12]), ("Node5", 'location', s[12:15])]
_mapping3 = lambda s: [("Node1", 'location', s[0:3]), ("Node2", 'location', s[3:6]), ("Node3", 'location', s[6:9])]
mapping = _mapping5
filename = "anim.txt"
speed = 1

def delete_all():
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete(use_global=False)

    for item in bpy.data.meshes:
       bpy.data.meshes.remove(item)

def run ():
    # Read the path file
    with open(filename) as fpath:
        path = [list(map(float, line.split())) for line in fpath.read().splitlines()]
        
    links = int(len(path[0]) / 3)
    
    origin = (0, 0, 0)
    
    delete_all()
    mat1 = bpy.data.materials.get("Node")
    mat2 = bpy.data.materials.get("Link")
    mat3 = bpy.data.materials.get("Shell")
    
    bpy.ops.mesh.primitive_uv_sphere_add(location = origin, segments = 64, ring_count = 32)
    bpy.ops.transform.resize(value=(links - 2, links - 2, links - 2))
    bpy.context.object.data.materials.append(mat3)
    
    for i in range(links + 1):
        bpy.ops.mesh.primitive_uv_sphere_add(location = origin)
        bpy.ops.transform.resize(value=(0.1, 0.1, 0.1))
        
        node = bpy.context.object
        node.name = "Node%d" % i
        for i in range(len(node.data.polygons)):
            node.data.polygons[i].use_smooth = True
        node.data.materials.append(mat1)
        
    for i in range(links):
        obj1 = bpy.data.objects["Node%d" % i]
        obj2 = bpy.data.objects["Node%d" % (i + 1)]
        ttc = obj1.constraints.new(type='TRACK_TO')
        ttc.target = obj2
        
        bpy.ops.mesh.primitive_cube_add(location = (0, 5, 0))
        bpy.ops.transform.resize(value=(0.25, 5, 0.25))
        bar = bpy.context.object
        bar.name = "Link%d" % i
        bar.data.materials.append(mat2)
        
        ttc = bar.constraints.new(type='CHILD_OF')
        ttc.target = obj1
        
            
        
    # Insert keyframe for every state
    f = 1.0
    i = 0
    for i in range(len(path)):
        if i % 2:
            continue
        line = path[i]
        for i in range(1, links + 1):
            obj = bpy.data.objects["Node%d" % i]
            obj.location = tuple(line[3 * (i - 1) : 3 * i])
            obj.keyframe_insert(data_path = 'location', frame = f)
        f += 1

    # Set the framerate for rendering
    bpy.context.scene.frame_end = f + 30
    bpy.context.scene.frame_step = speed
    bpy.context.scene.render.fps = 60

run()