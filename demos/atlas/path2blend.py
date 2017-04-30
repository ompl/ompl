import bpy, bmesh
import os
import math

def sign(n):
    return 1 if n >= 0 else -1

def vert_generator():
    data = bpy.context.object.data
    bm = bmesh.new()
    bm.from_mesh(data)
    for v in bm.verts:
        yield v

    bm.to_mesh(data)

def make_grid(loc, x_width, y_width, x_sub, y_sub):
    bpy.ops.mesh.primitive_grid_add(location = (0, 0, 0),
                                    x_subdivisions = x_sub,
                                    y_subdivisions = y_sub)

    for v in vert_generator():
        v.co.x *= x_width
        v.co.y *= y_width

        v.co.x += loc[0]
        v.co.y += loc[1]
        v.co.z = loc[2]

def smooth_shading():
    data = bpy.context.object.data
    for i in range(len(data.polygons)):
        data.polygons[i].use_smooth = True

def assign_mat(name):
    mat = bpy.data.materials.get(name)
    bpy.context.object.data.materials.append(mat)

class Wall:
    def ring_radius(self, v):
        return math.sqrt(self.radius * self.radius - v.co.x * v.co.x)

    def __init__(self, radius, offset, thickness, width, kind):
        width += 0.2
        self.radius = radius

        # Create obstacle mesh
        make_grid((offset, 0, 0), thickness, radius, 32, 128)
        smooth_shading()

        for v in vert_generator():
            rr = self.ring_radius(v)
            if abs(v.co.y) >= rr:
                v.co.y = rr * sign(v.co.y)

            v.co.z = math.sqrt(abs(rr * rr - v.co.y * v.co.y))

            if kind == 'l' and v.co.y > 0 and v.co.z <= width:
                v.co.z = width
                v.co.y = self.ring_radius(v) * sign(v.co.y)

            if kind == 'r' and v.co.y < 0 and v.co.z <= width:
                v.co.z = width
                v.co.y = self.ring_radius(v) * sign(v.co.y)

            v.co.y += 0.05 * sign(v.co.y)
            v.co.z += 0.05

            if v.co.z < 0.1:
                v.co.z = -0.1

        assign_mat("Obstacle")


def make_walls(size):
    bpy.ops.mesh.primitive_cube_add(location = (0, 0, -0.2))
    bpy.ops.transform.resize(value=(size, size, 0.1))
    assign_mat("Box")

    bpy.ops.mesh.primitive_cube_add(location = (0, size, 0))
    bpy.ops.transform.resize(value=(size, 0.1, size))
    assign_mat("Box")

    bpy.ops.mesh.primitive_cube_add(location = (size, 0, 0))
    bpy.ops.transform.resize(value=(0.1, size, size))
    assign_mat("Box")

    bpy.ops.mesh.primitive_cube_add(location = (-size, 0, 0))
    bpy.ops.transform.resize(value=(0.1, size, size))
    assign_mat("Box")

def make_shell(radius):
    bpy.ops.mesh.primitive_uv_sphere_add(location = (0, 0, 0),
                                         segments = 64,
                                         ring_count = 32)
    smooth_shading()

    bpy.ops.transform.resize(value = (radius, radius, radius))
    assign_mat("Shell")

def delete_all():
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete(use_global=False)

    for item in bpy.data.meshes:
       bpy.data.meshes.remove(item)

def run (filename):
    with open(filename) as f:
        path = [list(map(float, line.split())) for line in f.read().splitlines()][:-1]

    links = int(len(path[0]) / 3)
    radius = links - 2

    delete_all()
    make_shell(radius)
    make_walls(links)

    node_mat = bpy.data.materials.get("Node")
    link_mat = bpy.data.materials.get("Link")

    obstacles = 2
    step = 2 * radius / float(obstacles + 1)
    current = -radius + step
    for i in range(obstacles):
        mode = 'l' if i % 2 else 'r'
        Wall(radius, current, 0.25, 0.5, mode)
        current += step

    scene = bpy.data.scenes["Scene"]
    scene.camera.location.x = 0
    scene.camera.location.y = -2 * (links + 1)
    scene.camera.location.z = links + 1

    for i in range(links + 1):
        bpy.ops.mesh.primitive_uv_sphere_add(location = (0, 0, 0))
        bpy.ops.transform.resize(value=(0.1, 0.1, 0.1))

        node = bpy.context.object
        node.name = "Node%d" % i
        for i in range(len(node.data.polygons)):
            node.data.polygons[i].use_smooth = True
        node.data.materials.append(node_mat)

    for i in range(links):
        obj1 = bpy.data.objects["Node%d" % i]
        obj2 = bpy.data.objects["Node%d" % (i + 1)]
        ttc = obj1.constraints.new(type='TRACK_TO')
        ttc.target = obj2

        bpy.ops.mesh.primitive_cube_add(location = (0, 5, 0))
        bpy.ops.transform.resize(value=(0.25, 5, 0.25))
        bar = bpy.context.object
        bar.name = "Link%d" % i
        bar.data.materials.append(link_mat)

        ttc = bar.constraints.new(type='CHILD_OF')
        ttc.target = obj1

    # Insert keyframe for every state
    f = 10.0
    i = 0
    for i in range(len(path)):
        line = path[i]
        for i in range(1, links + 1):
            obj = bpy.data.objects["Node%d" % i]
            obj.location = tuple(line[3 * (i - 1) : 3 * i])
            obj.keyframe_insert(data_path = 'location', frame = f)
        f += 3

    # Set the framerate for rendering
    bpy.context.scene.frame_end = f + 30
    bpy.context.scene.frame_step = 1
    bpy.context.scene.render.fps = 60

if __name__ == "__main__":
    run('/home/zak/lab/code/ompl/build/anim.txt')
