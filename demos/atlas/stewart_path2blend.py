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

def getNode(line, chain, link):
    global links
    offset = chain * links * 3
    return mathutils.Vector(line[offset + 3 * (link - 1) : offset + 3 * link])

def normal(p1, p2, p3):
    v = p2 - p1
    w = p3 - p1
    
    return v.cross(w)
    

# Read the path file
with open(filename) as fpath:
    path = [list(map(float, line.split())) for line in fpath.read().splitlines()][:-1]

links = 0
for i in range(1, int(len(path[-1]) / 3.)):
    x = getNode(path[-1], 0, i)
    if int(x[0] - 1) == 0 and int(x[2] - 1) == 0:
        links = i
        break

links = 5
    
chains = int(len(path[0][2::3]) / links)
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

bpy.ops.mesh.primitive_cylinder_add(location = (0, 0, path[0][3 * links - 1]))
bpy.ops.transform.resize(value=(0.9, 0.9, 0.1))
disk = bpy.context.object
disk.name = "Centroid"
disk.data.materials.append(link_mat)
disk.rotation_mode = 'QUATERNION'

for j in range(chains):
    offset = j * links * 3 + links * 3 - 3
    obj = bpy.data.objects["Chain%dNode0" % j]
    obj.location = tuple(path[0][offset: offset + 2] + [0])

# Insert keyframe for every state
f = 10.0
i = 0
for i in range(len(path)):
    line = path[i]
    if not line:
        break

    centroid = mathutils.Vector([0, 0, 0])
    for j in range(chains):
        for i in range(1, links + 1):
            obj = bpy.data.objects["Chain%dNode%d" % (j, i)]
            obj.location = getNode(line, j, i)
            obj.keyframe_insert(data_path = 'location', frame = f)

        centroid += getNode(line, j, links) / chains

    obj = bpy.data.objects["Centroid"]
    obj.location = centroid
    obj.keyframe_insert(data_path = 'location', frame = f)
    
    if chains == 2:
        centroid = (getNode(line, 0, links) + getNode(line, 1, links)) / 2 + mathutils.Vector([0, 1, 0])
        obj.rotation_quaternion = normal(getNode(line, 0, links),
                                         getNode(line, 1, links),
                                         centroid).to_track_quat('Z', 'Y')
    else:
        obj.rotation_quaternion = normal(getNode(line, 0, links),
                                        getNode(line, int(chains / 3), links),
                                        getNode(line, int(2 * chains / 3), links)
                                        ).to_track_quat('Z', 'Y')
      
    obj.keyframe_insert(data_path = 'rotation_quaternion', frame = f)
    f += 3

# Set the framerate for rendering
bpy.context.scene.frame_end = f + 30
bpy.context.scene.frame_step = 1
bpy.context.scene.render.fps = 60
