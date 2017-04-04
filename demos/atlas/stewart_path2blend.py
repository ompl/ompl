import bpy
import math
import mathutils
import os

os.chdir('/home/zak/lab/code/ompl/build/')

filename = "anim.txt"

def delete_all():
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete(use_global=False)

    for item in bpy.data.meshes:
       bpy.data.meshes.remove(item)

def make_box(box):
    box_mat = bpy.data.materials.get("Box")
    
    box = 20

    bpy.ops.mesh.primitive_cube_add(location = (0, 0, -0.2))
    bpy.ops.transform.resize(value=(box, box, 0.1))
    bpy.context.object.data.materials.append(box_mat)

    bpy.ops.mesh.primitive_cube_add(location = (0, box, 0))
    bpy.ops.transform.resize(value=(box, 0.1, box))
    bpy.context.object.data.materials.append(box_mat)

    bpy.ops.mesh.primitive_cube_add(location = (box, 0, 0))
    bpy.ops.transform.resize(value=(0.1, box, box))
    bpy.context.object.data.materials.append(box_mat)
    return

    bpy.ops.mesh.primitive_cube_add(location = (-box, 0, 0))
    bpy.ops.transform.resize(value=(0.1, box, box))
    bpy.context.object.data.materials.append(box_mat)
    
def cross(a, b, c):
    ab = [y - x for x, y in zip(a, b)]
    ac = [y - x for x, y in zip(a, c)]
    
    return [ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0]]

def run ():
    # Read the path file
    with open(filename) as fpath:
        path = [list(map(float, line.split())) for line in fpath.read().splitlines()]

    chains = 4
    links = int(len(path[0][2::3]) / chains)
    radius = path[0][0]

    origin = (0, 0, 0)

    delete_all()

    make_box(radius + 2)

    scene = bpy.data.scenes["Scene"]
    scene.camera.location.x = -2 * (radius + 4)
    scene.camera.location.y = -2 * (radius + 4)
    scene.camera.location.z = radius + 4

    # Make the chains
    node_mat = bpy.data.materials.get("Node")
    for i in range(links + 1):
        for j in range(chains):
            bpy.ops.mesh.primitive_uv_sphere_add(location = origin)
            bpy.ops.transform.resize(value=(0.1, 0.1, 0.1))

            node = bpy.context.object
            node.name = "Chain%dNode%d" % (j, i)
            for k in range(len(node.data.polygons)):
                node.data.polygons[k].use_smooth = True
            node.data.materials.append(node_mat)

    link_mat = bpy.data.materials.get("Link")
    for i in range(links):
        for j in range(chains):
            obj1 = bpy.data.objects["Chain%dNode%d" % (j, i)]
            obj2 = bpy.data.objects["Chain%dNode%d" % (j, i + 1)]
            ttc = obj1.constraints.new(type='TRACK_TO')
            ttc.target = obj2

            bpy.ops.mesh.primitive_cube_add(location = (0, 5, 0))
            bpy.ops.transform.resize(value=(0.25, 5, 0.25))
            bar = bpy.context.object
            bar.name = "Chain%dLink%d" % (j, i)
            bar.data.materials.append(link_mat)

            ttc = bar.constraints.new(type='CHILD_OF')
            ttc.target = obj1
            
    for j in range(chains):
        offset = j * links * 3 + links * 3 - 3
        obj = bpy.data.objects["Chain%dNode0" % j]
        obj.location = tuple(path[0][offset: offset + 2] + [0])
        
    bpy.ops.mesh.primitive_cylinder_add(location = (0, 0, path[0][3 * links - 1]))
    bpy.ops.transform.resize(value=(0.9, 0.9, 0.1))
    bar = bpy.context.object
    bar.name = "Disk"
    bar.data.materials.append(link_mat)
    bar.rotation_mode = 'AXIS_ANGLE'
    
    # Insert keyframe for every state
    f = 10.0
    i = 0
    for i in range(len(path)):
        line = path[i]
        if not line:
            break
        
        centroid = [0, 0, 0]
        for j in range(chains):
            offset = j * links * 3
            for i in range(1, links + 1):
                obj = bpy.data.objects["Chain%dNode%d" % (j, i)]
                obj.location = tuple(line[offset + 3 * (i - 1) : offset + 3 * i])
                obj.keyframe_insert(data_path = 'location', frame = f)
            centroid[0] += line[(j + 1) * links * 3 - 3] / chains
            centroid[1] += line[(j + 1) * links * 3 - 2] / chains
            centroid[2] += line[(j + 1) * links * 3 - 1] / chains
                
        obj = bpy.data.objects["Disk"]
        
        obj.location = tuple(centroid)
        obj.keyframe_insert(data_path = 'location', frame = f)
        
        vs = []
        for i in range(3):
            idx = int(chains / 3 * i) % chains
            vs.append(mathutils.Vector(line[(idx + 1) * links * 3 - 3 : (idx + 1) * links * 3]))
            
        vs1 = vs[0] - vs[1]
        vs2 = vs[0] - vs[2]
        vs1.normalize(); vs2.normalize()
        vs3 = vs1.cross(vs2)
        vs3.normalize()
        
        vs4 = mathutils.Vector([1, 0, 0])
        vs5 = vs3.cross(vs4)
        
        obj.rotation_axis_angle = tuple([math.pi /2 - math.acos(vs4.dot(vs3)), vs5.x, vs5.y, vs5.z])
        obj.keyframe_insert(data_path = 'rotation_axis_angle', frame = f)
        f += 2

    # Set the framerate for rendering
    bpy.context.scene.frame_end = f + 30
    bpy.context.scene.frame_step = 1
    bpy.context.scene.render.fps = 60

run()
