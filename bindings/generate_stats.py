import os 
import re 


def get_files_in_dir(path, endwith=".cpp", recursive=False):
    all_files = []
    if recursive:
        for root, dirs, files in os.walk(path):
            for file in files:
                if file.endswith(endwith):
                    all_files.append(os.path.join(root, file))
    else:
        for file in os.listdir(path):
            if file.endswith(endwith):
                all_files.append(os.path.join(path, file))
    return all_files

def inspect_file(file_path):
    pattern_class_name = re.compile(r'nb::class_<[^>]+>\(m,\s*"([^"]+)"\)')
    pattern_todo_note = re.compile(r'//\s*TODO\s*\[([^]]+)\]\[([^]]+)\]')
    class_names = []
    class_todo_implement = []
    class_todo_test = []
    class_todo_trampoline = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            class_match = pattern_class_name.search(line)
            if class_match:
                class_name = class_match.group(1)
                class_names.append(class_name)
            
            todo_match = pattern_todo_note.search(line)
            if todo_match:
                class_name = todo_match.group(1)
                todo_type = todo_match.group(2)
                if todo_type == "IMPLEMENT":
                    class_todo_implement.append(class_name)
                elif todo_type == "TEST":
                    class_todo_test.append(class_name)
                elif todo_type == "TRAMPOLINE":
                    class_todo_trampoline.append(class_name)
        
    return class_names, class_todo_implement, class_todo_test, class_todo_trampoline


def write_list_png(lines, font_size=6, width=6, line_spacing=0.3):
    import matplotlib.pyplot as plt
    """
    Render a list of strings into a PNG image, one per line.
    """
    line_height_in = (font_size / 72) * line_spacing
    height_in = max(len(lines) * line_height_in, line_height_in)

    fig, ax = plt.subplots(figsize=(width, height_in))
    ax.axis('off')

    text = "\n".join(lines)
    # draw in axes coords so (0,1) is truly top‐left
    ax.text(0, 1, text,
            va='top', ha='left',
            fontsize=font_size,
            family='monospace',
            transform=ax.transAxes)

    # remove all margins
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    return fig


bindings_path = os.path.dirname(os.path.abspath(__file__))

class_total_count = 0
class_todo_implement_count = 0
class_todo_test_count = 0
class_todo_trampoline_count = 0

base_todo_implement = []
base_todo_test = []
all_files_in_base = get_files_in_dir(os.path.join(bindings_path, "base"), endwith=".cpp", recursive=True)
for file in all_files_in_base:
    class_names, class_todo_implement, class_todo_test, class_todo_trampoline = inspect_file(file)
    class_total_count += len(class_names)
    class_todo_implement_count += len(class_todo_implement)
    class_todo_test_count += len(class_todo_test) + len(class_todo_implement)
    class_todo_trampoline_count += len(class_todo_trampoline)
    base_todo_implement.extend(class_todo_implement)
    base_todo_test.extend(class_todo_test)
print(f"Base TODO [IMPLEMENT]: {base_todo_implement}")
fig = write_list_png(base_todo_implement)
fig.savefig(os.path.join(bindings_path, "stats", "base_todo_implement.png"), bbox_inches='tight', dpi=300)

geometric_todo_implement = []
geometric_todo_test = []
all_files_in_geometric = get_files_in_dir(os.path.join(bindings_path, "geometric"), endwith=".cpp", recursive=True)
for file in all_files_in_geometric:
    class_names, class_todo_implement, class_todo_test, class_todo_trampoline = inspect_file(file)
    class_total_count += len(class_names)
    class_todo_implement_count += len(class_todo_implement)
    class_todo_test_count += len(class_todo_test) + len(class_todo_implement)
    class_todo_trampoline_count += len(class_todo_trampoline)
    geometric_todo_implement.extend(class_todo_implement)
    geometric_todo_test.extend(class_todo_test)
print(f"Geometric TODO [IMPLEMENT]: {geometric_todo_implement}")
fig = write_list_png(geometric_todo_implement)
fig.savefig(os.path.join(bindings_path, "stats", "geometric_todo_implement.png"), bbox_inches='tight', dpi=300)

control_todo_implement = []
control_todo_test = []
all_files_in_control = get_files_in_dir(os.path.join(bindings_path, "control"), endwith=".cpp", recursive=True)
for file in all_files_in_control:
    class_names, class_todo_implement, class_todo_test, class_todo_trampoline = inspect_file(file)
    class_total_count += len(class_names)
    class_todo_implement_count += len(class_todo_implement)
    class_todo_test_count += len(class_todo_test) + len(class_todo_implement)
    class_todo_trampoline_count += len(class_todo_trampoline)
    control_todo_implement.extend(class_todo_implement)
    control_todo_test.extend(class_todo_test)
print(f"Control TODO [IMPLEMENT]: {control_todo_implement}")
fig = write_list_png(control_todo_implement)
fig.savefig(os.path.join(bindings_path, "stats", "control_todo_implement.png"), bbox_inches='tight', dpi=300)

print(f"Total classes: {class_total_count}")
print(f"Classes with TODO [IMPLEMENT]: {class_todo_implement_count}")
print(f"Classes with TODO [TEST]: {class_todo_test_count}")
print(f"Classes with TODO [TRAMPOLINE]: {class_todo_trampoline_count}")

def inspect_planner_file(file_path):
    pattern_todo_note = re.compile(r'//\s*TODO\s*\[([^]]+)\]\[([^]]+)\]')
    pattern_tag_note = re.compile(r'//\s*TAG\s*\[([^]]+)\]\[([^]]+)\]')

    planner_tag = False
    todo_tag = False
    planner_name = None
    todo_names = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            todo_match = pattern_todo_note.search(line)
            tag_match = pattern_tag_note.search(line)
            if tag_match:
                planner_name = tag_match.group(1)
                planner_tag = True
            if todo_match:
                todo_names.append(todo_match.group(1))
                todo_tag = True

    if planner_name not in todo_names:
        todo_tag = False

    if planner_tag and not todo_tag:
        # Planner is done
        return planner_name, None
    elif planner_tag and todo_tag:
        return None, planner_name
    else:
        # No planner tag found
        return None, None
all_geometric_planner_done = []
all_geometric_planner_todo = []

all_files_in_geometric_planner = get_files_in_dir(os.path.join(bindings_path, "geometric", "planners"), endwith=".cpp", recursive=True)
for file in all_files_in_geometric_planner:
    planner_done, planner_todo = inspect_planner_file(file)
    if planner_done:
        all_geometric_planner_done.append(planner_done)
    if planner_todo:
        all_geometric_planner_todo.append(planner_todo)

print(f"Total planners done: {all_geometric_planner_done}")
print(f"Total planners todo: {all_geometric_planner_todo}")
write_list_png(all_geometric_planner_done).savefig(os.path.join(bindings_path, "stats", "geometric_planners_done.png"), bbox_inches='tight', dpi=300)
write_list_png(all_geometric_planner_todo).savefig(os.path.join(bindings_path, "stats", "geometric_planners_todo.png"), bbox_inches='tight', dpi=300)

all_control_planner_done = []
all_control_planner_todo = []
all_files_in_control_planner = get_files_in_dir(os.path.join(bindings_path, "control", "planners"), endwith=".cpp", recursive=True)
for file in all_files_in_control_planner:
    planner_done, planner_todo = inspect_planner_file(file)
    if planner_done:
        all_control_planner_done.append(planner_done)
    if planner_todo:
        all_control_planner_todo.append(planner_todo)
print(f"Total control planners done: {all_control_planner_done}")
print(f"Total control planners todo: {all_control_planner_todo}")
write_list_png(all_control_planner_done).savefig(os.path.join(bindings_path, "stats", "control_planners_done.png"), bbox_inches='tight', dpi=300)
write_list_png(all_control_planner_todo).savefig(os.path.join(bindings_path, "stats", "control_planners_todo.png"), bbox_inches='tight', dpi=300)
def plot_stats():
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Wedge

    # ── Data ──
    total   = class_total_count
    metrics = {
        'Implemented': class_total_count - class_todo_implement_count,
        'Tested':      class_total_count - class_todo_test_count,
    }

    # ── Color mapping ──
    def pct_color(p):
        if   p < 50:  return '#e74c3c'   # red
        elif p < 90:  return '#f1c40f'   # orange
        else:         return '#2ecc71'   # green

    fig, axes = plt.subplots(1, 2, figsize=(8, 4), constrained_layout=True)
    for ax, (label, count) in zip(axes, metrics.items()):
        pct   = count / total * 100
        theta = pct / 100 * 360
        color = pct_color(pct)

        # full grey ring (background)
        ring = Circle((0, 0), 1, fill=False, linewidth=15, edgecolor='#dddddd', zorder=1)
        ax.add_patch(ring)

        # colored wedge arc from 12 o'clock, CCW
        arc = Wedge((0, 0), 1, theta1=90, theta2=90 + theta,
                    width=0.15, facecolor=color, edgecolor='none', zorder=2)
        ax.add_patch(arc)

        # percentage text
        ax.text(0, 0, f"{pct:.0f}%", ha='center', va='center',
                fontsize=20, color='black', zorder=3)

        # label below
        ax.text(0, -1.3, f"{label}", ha='center', va='center',
                fontsize=12)

        # adjust view
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.4, 1.2)
        ax.set_aspect('equal')
        ax.axis('off')
    return fig

fig = plot_stats()
fig.savefig(os.path.join(bindings_path, "stats", "bindings_stats.png"), bbox_inches='tight', dpi=300)