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

bindings_path = os.path.dirname(os.path.abspath(__file__))

class_total_count = 0
class_todo_implement_count = 0
class_todo_test_count = 0
class_todo_trampoline_count = 0

all_files_in_base = get_files_in_dir(os.path.join(bindings_path, "base"), endwith=".cpp", recursive=True)
for file in all_files_in_base:
    class_names, class_todo_implement, class_todo_test, class_todo_trampoline = inspect_file(file)
    class_total_count += len(class_names)
    class_todo_implement_count += len(class_todo_implement)
    class_todo_test_count += len(class_todo_test) + len(class_todo_implement)
    class_todo_trampoline_count += len(class_todo_trampoline)

all_files_in_geometric = get_files_in_dir(os.path.join(bindings_path, "geometric"), endwith=".cpp", recursive=True)
for file in all_files_in_geometric:
    class_names, class_todo_implement, class_todo_test, class_todo_trampoline = inspect_file(file)
    class_total_count += len(class_names)
    class_todo_implement_count += len(class_todo_implement)
    class_todo_test_count += len(class_todo_test) + len(class_todo_implement)
    class_todo_trampoline_count += len(class_todo_trampoline)

all_files_in_control = get_files_in_dir(os.path.join(bindings_path, "control"), endwith=".cpp", recursive=True)
for file in all_files_in_control:
    class_names, class_todo_implement, class_todo_test, class_todo_trampoline = inspect_file(file)
    class_total_count += len(class_names)
    class_todo_implement_count += len(class_todo_implement)
    class_todo_test_count += len(class_todo_test) + len(class_todo_implement)
    class_todo_trampoline_count += len(class_todo_trampoline)

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
all_planner_done = []
all_planner_todo = []

all_files_in_planner = get_files_in_dir(os.path.join(bindings_path, "geometric", "planners"), endwith=".cpp", recursive=True)
for file in all_files_in_planner:
    planner_done, planner_todo = inspect_planner_file(file)
    if planner_done:
        all_planner_done.append(planner_done)
    if planner_todo:
        all_planner_todo.append(planner_todo)

print(f"Total planners done: {all_planner_done}")
print(f"Total planners todo: {all_planner_todo}")

