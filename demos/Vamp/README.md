# OMPL-VAMP Integration Demo

## Prerequisites
- Python 3.7+ with numpy, pybullet for visualization

## Usage
```bash
./demo_Vamp                    # Basic programmatic example
./demo_Vamp --visualize        # Basic example + visualization  
./demo_Vamp config.yaml        # YAML configuration mode
```

**Custom Robot Support**
```bash
./demo_CustomRobot                                    # Custom robot demo
./demo_CustomRobot --list                             # List all registered robots  
./demo_CustomRobot --robot planar_arm_2dof            # Specific custom robot
./demo_Vamp demos/Vamp/config/planar_arm_2dof_demo.yaml     # YAML mode with custom robot
```

**Visualization (URDF-Configurable)**
```bash
# Automatic visualization with embedded URDF configuration
python3 visualize_solution.py solution_path_file.txt

# Manual override
python3 visualize_solution.py solution_path_file.txt --robot panda --yaml-config config.yaml
```

## Options
- `--visualize`: Enable 3D visualization
- `--help`: Show usage information
- `config.yaml`: Use YAML configuration file
- `--list`: List all registered robots (including custom ones)
- `--robot <name>`: Select specific robot
- `--info <name>`: Show robot information

## Examples

### Built-in Robots
Plan Panda 7-DOF arm through sphere obstacles:
```bash
./demo_Vamp demos/Vamp/config/panda_demo.yaml
pip install -r demos/Vamp/requirements.txt  # For visualization
```

## Features
- SIMD-accelerated collision checking via VAMP
- Multiple OMPL planners (RRT-Connect, RRT*, BiT*)
- YAML-based scene configuration
- Interactive 3D visualization with PyBullet
- Support for multiple robot types (Panda, UR5, Fetch)
- Custom robot registration
- Pointcloud obstacle support** (.xyz, .ply, .pcd formats)

## Custom Robot Development

### Registering New Robots

The registry system allows you to add custom robots without modifying core code:

1. **Define your robot** following the VAMP robot interface:
```cpp
namespace vamp::robots {
    struct MyCustomRobot {
        static constexpr auto name = "my_robot";
        static constexpr auto dimension = 6;  // 6-DOF
        static constexpr auto n_spheres = 10; // Collision spheres
        static constexpr auto resolution = 32;
        
        // Joint limits
        static constexpr std::array<float, dimension> s_a = {/*...*/};
        static constexpr std::array<float, dimension> s_m = {/*...*/};
        
        // Required methods
        template<std::size_t rake>
        inline static auto fkcc(const Environment<FloatVector<rake>>& env,
                               const ConfigurationBlock<rake>& q) -> bool {
            // Implement vectorized forward kinematics + collision checking
        }
        
        // ... other required methods
    };
}
```

2. **Register your robot**:
```cpp
#include "VampRobotRegistry.h"
REGISTER_VAMP_ROBOT(vamp::robots::MyCustomRobot, "my_robot");
```

3. **Use in YAML configuration**:
```yaml
robot:
  name: "my_robot"
  description: "My Custom Robot"

start_config: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_config: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
# ... rest of configuration
```

### Example Custom Robots

The demo includes a example custom robot:

- **planar_arm_2dof**: Simple 2-DOF planar manipulator for learning

See `CustomRobotExample.h` for complete implementation details.

## Pointcloud Support
The demo supports pointcloud obstacles alongside primitive shapes:

```yaml
obstacles:
  - type: pointcloud
    name: "environment_pointcloud"
    pointcloud_file: "path/to/pointcloud.xyz"  # Supports .xyz, .ply, .pcd
    point_radius: 0.0025  # Radius for each point in CAPT construction
```

Supported formats:
- `.xyz`: Simple ASCII format (x y z per line)
- `.ply`: Stanford PLY format (ASCII only)
- `.pcd`: Point Cloud Data format (ASCII only)

## Notes
Visualization automatically detects robot type from planning configuration.
Pointclouds use VAMP's CAPT (Collision Affording Point tree) Data Structure
Custom robots require implementation of vectorized forward kinematics for optimal performance. 