import bpy

# Syntax for each mapping entry is ("object name", "property name", [list of values])
#  where a value of None indicates no change is to be made.
mapping = lambda s: [("Node1", 'location', s[0:3]), ("Node2", 'location', s[3:6]), ("Node3", 'location', s[6:9]),
                     ("Node4", 'location', s[9:12]), ("Node5", 'location', s[12:15])]
filename = "anim.txt"
speed = 2

def run ():
    # Read the path file
    with open(filename) as fpath:
        path = fpath.read().splitlines()

    # Set up object rotation modes as inferred from the mapping entries; integrity check the mapping; clear previous keyframes
    for m in mapping(range(len(path[0].split()))):
        assert len(m) == 3, "Mapping entry " + str(m) + " must be a 3-tuple."
        o = bpy.data.objects[m[0]]
        o.animation_data_clear()
        if m[1] == 'rotation_quaternion':
            o.rotation_mode = 'QUATERNION'
            assert len(m[2]) == 4, "In mapping entry " + str(m) + ", rotation requires 4 values (found " + str(len(m[2])) + ")."
        elif m[1] == 'rotation_euler':
            o.rotation_mode = 'XYZ'
            assert len(m[2]) == 3, "In mapping entry " + str(m) + ", rotation requires 3 values (found " + str(len(m[2])) + ")."
        elif m[1] == 'location':
            assert len(m[2]) == 3, "In mapping entry " + str(m) + ", location requires 3 values (found " + str(len(m[2])) + ")."

    # Insert keyframe for every state
    f = 1.0
    for line in path:
        state = list(map(float, line.split()))
        for m in mapping(state):
            o = bpy.data.objects[m[0]]
            prop = getattr(o, m[1])
            assert len(prop) == len(m[2]), "Final value in mapping entry " + str(m) + " should be the same length as the property " + str(prop) + "."
            setattr(o, m[1], list(map(lambda i: m[2][i] if m[2][i] != None else prop[i], range(len(m[2])))))
            o.keyframe_insert(data_path=m[1], frame=f)
        f += 1

    # Set the framerate for rendering
    bpy.context.scene.frame_end = f+30
    bpy.context.scene.frame_step = speed
    bpy.context.scene.render.fps = 30
