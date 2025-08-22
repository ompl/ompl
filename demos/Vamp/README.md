# OMPL-VAMP Integration Demo

## Prerequisites
- pybullet for visualization

## Demo Usage

### **Basic Usage**
```bash
# Single planning run (default mode)
./demo_Vamp                              # Programmatic example
./demo_Vamp panda_demo.yaml              # YAML configuration

# Benchmarking mode  
./demo_Vamp panda_benchmark.yaml --benchmark  # Multi-run benchmark with OMPL output
./demo_Vamp --robot panda                    # Quick benchmark for Panda robot
./demo_Vamp --robot ur5                      # Quick benchmark for UR5 robot
./demo_Vamp --robot fetch                    # Quick benchmark for Fetch robot

# Visualization mode
./demo_Vamp panda_demo.yaml --visualize  # Single run + 3D visualization

# Utility options
./demo_Vamp --list-robots                # Show available robots
./demo_Vamp --help                       # Full usage information
```

### **Advanced Benchmarking**
```bash
# Custom benchmark parameters
./demo_Vamp config.yaml --benchmark --runs 50 --timeout 10.0 --experiment "Custom Test"

# Quick robot testing for all supported robots
./demo_Vamp --robot panda                    # 10 runs, 3 second timeout
./demo_Vamp --robot ur5                      # 10 runs, 3 second timeout  
./demo_Vamp --robot fetch                    # 10 runs, 3 second timeout
```

### **Custom Robot Support**
```bash
./demo_Vamp --planar-arm                      # 2DOF planar arm demo
./demo_Vamp --planar-arm --visualize          # With visualization
./demo_Vamp --robot planar_arm_2dof --benchmark  # Benchmark custom robot
```

### **Visualization**
```bash
# Automatic visualization with embedded URDF configuration
python3 visualize_solution.py solution_path_file.txt

# Manual override
python3 visualize_solution.py solution_path_file.txt --robot panda --yaml-config config.yaml
```

##  Demo Options
- `--benchmark`: Enable benchmarking mode (generates OMPL-compliant .log files)
- `--visualize`: Enable 3D visualization output
- `--robot <name>`: Run quick benchmark for specific robot (panda, ur5, fetch)
- `--runs <N>`: Number of benchmark runs (default: 25)
- `--timeout <seconds>`: Planning timeout per run (default: 5.0)
- `--experiment <name>`: Custom experiment name for benchmarks
- `--robot <name>`: Select specific robot for quick tests or benchmarking
- `--list-robots`: List all registered robots
- `--help`: Show complete usage information

## Examples

### Built-in Robots
Plan Panda 7-DOF arm through sphere obstacles:
```bash
./demo_Vamp demos/Vamp/config/panda_demo.yaml
pip install -r demos/Vamp/requirements.txt  # For visualization
```

## Features
- SIMD-accelerated collision checking via VAMP
- OMPL planners
- YAML-based scene configuration
- Interactive 3D visualization with PyBullet
- Support for multiple robot types (Panda, UR5, Fetch)
- Custom robot registration
- Pointcloud obstacle support (.xyz, .ply, .pcd formats)
- **OMPL-compliant benchmarking** with Planner Arena integration

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

### Custom Robot Benchmarking

Custom robots automatically support benchmarking through the registry system:

```bash
# Benchmark custom robots
./demo_Vamp --robot planar_arm_2dof --benchmark
./demo_Vamp --robot panda --benchmark
./demo_Vamp --robot ur5 --benchmark

# List robots with benchmarking support
./demo_Vamp --list-robots
```

All registered robots (built-in and custom) automatically get benchmarking capabilities through the `VampRobotRegistry` integration.

## Benchmarking System

VAMP includes **comprehensive benchmarking** built into the main demo, generating standard OMPL log files compatible with `ompl_benchmark_statistics.py` and Planner Arena. The benchmarking system is now fully integrated with the `VampRobotRegistry`, enabling benchmarking for all registered robots.

### Registry-Based Benchmarking
All robots registered in the `VampRobotRegistry` automatically support benchmarking:

```bash
# Quick benchmarking for any registered robot
./demo_Vamp --robot panda                    # Panda 7-DOF robot
./demo_Vamp --robot ur5                      # UR5 6-DOF robot  
./demo_Vamp --robot fetch                    # Fetch 8-DOF robot

# Full benchmarking from YAML configuration
./demo_Vamp panda_demo.yaml --benchmark      # Full benchmark from YAML
./demo_Vamp config.yaml --benchmark --runs 100  # Custom run count
```

### Robot Registry Integration
The benchmarking system leverages the `VampRobotRegistry` for:
- **Automatic robot detection** - All registered robots support benchmarking
- **Type-safe operations** - Proper robot-specific configurations
- **Extensible design** - New robots automatically get benchmarking support
- **Consistent interface** - Same API for all robots

### Complete Workflow: Benchmark → Database → Visualization
```bash
# 1. Run benchmark (generates OMPL-compliant .log file)
./demo_Vamp panda_benchmark.yaml --benchmark

# 2. Generate SQLite database  
python3 scripts/ompl_benchmark_statistics.py vamp_benchmark_panda_*.log -d benchmark.db

# 3. Visualize results at http://plannerarena.org
# Simply upload benchmark.db for interactive analysis
```

### YAML Benchmark Configuration
```yaml
benchmark:
  experiment_name: "Panda 7-DOF Comparison"
  runs: 50
  timeout: 5.0
  planners:
    - name: "RRT-Connect"
    - name: "BIT*"
    - name: "PRM"
```

**Key Benefits:**
-  **Single unified interface** - one demo for planning, benchmarking, and visualization
-  **Standard OMPL log format** - fully compatible with existing OMPL tools
-  **Planner Arena integration** - upload results for web visualization  
-  **Database generation** - use standard `ompl_benchmark_statistics.py`
-  **SIMD performance** - leverages VAMP's vectorized collision detection
-  **Progressive enhancement** - start with single runs, scale to benchmarks

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
- Visualization automatically detects robot type from planning configuration.
- Pointclouds use VAMP's CAPT (Collision Affording Point tree) Data Structure
- Custom robots require implementation of vectorized forward kinematics for optimal performance.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

The VAMP integration package is distributed under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0). You may obtain a copy of the License at:

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License. 