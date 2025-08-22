# VAMP-OMPL Integration Demo

A high-performance motion planning system that integrates VAMP's SIMD-accelerated collision detection with OMPL's comprehensive planner library.

## Quick Start

### Prerequisites
- pybullet (for visualization): `pip install pybullet`

### Basic Usage
```bash
# Single planning run
./demo_Vamp                              # Default configuration
./demo_Vamp panda_demo.yaml              # YAML configuration

# Benchmarking
./demo_Vamp panda_benchmark.yaml --benchmark
./demo_Vamp --robot panda               # Quick benchmark

# Visualization
./demo_Vamp panda_demo.yaml --visualize
```

### Command Line Options
- `--benchmark`: Enable benchmarking mode (generates OMPL-compliant .log files)
- `--visualize`: Enable 3D visualization output
- `--robot <name>`: Quick benchmark for specific robot (panda, ur5, fetch)
- `--planar-arm`: Run 2DOF planar arm demo (can combine with --benchmark or --visualize)
- `--list-robots`: List all registered robots
- `--help`: Show complete usage information

## Key Features

- **SIMD-Accelerated Performance**: 8x faster collision checking via vectorization
- **Extensible OMPL Planners**: Built-in support for RRT-Connect, BIT*, PRM with easy registration for additional planners
- **Multiple Robot Types**: Built-in support for Panda, UR5, Fetch robots
- **Custom Robot Support**: Extensible robot registration system
- **YAML Configuration**: Flexible scene and planner configuration
- **OMPL-Compliant Benchmarking**: Standard .log files compatible with Planner Arena
- **3D Visualization**: Interactive PyBullet-based visualization
- **Pointcloud Support**: .xyz, .ply, .pcd obstacle formats

## Configuration Examples

### YAML Configuration
```yaml
robot:
  name: "panda"
  description: "Franka Emika Panda arm"

planner:
  name: "RRT-Connect"
  planning_time: 5.0

start_config: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_config: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

obstacles:
  - type: "sphere"
    position: [0.5, 0.5, 0.5]
    radius: 0.1
  - type: pointcloud
    pointcloud_file: "environment.xyz"
    point_radius: 0.0025
```

### Benchmarking Workflow
```bash
# 1. Run benchmark
./demo_Vamp panda_benchmark.yaml --benchmark

# 2. Generate database (from OMPL root directory)
python3 scripts/ompl_benchmark_statistics.py vamp_benchmark_*.log -d results.db

# 3. Visualize at http://plannerarena.org (upload results.db)
```

## Custom Robot Development

For adding new robots, see the complete guide in [`docs/CONTRIBUTOR-GUIDE.md`](docs/CONTRIBUTOR-GUIDE.md#custom-robot-development).

**Quick Example:**
```cpp
// 1. Define robot (see examples/CustomRobotExample.h)
namespace vamp::robots {
    struct MyRobot {
        static constexpr auto name = "my_robot";
        static constexpr auto dimension = 6;
        // ... implement required interface
    };
}

// 2. Register robot
REGISTER_VAMP_ROBOT(vamp::robots::MyRobot, "my_robot");

// 3. Use in YAML or command line
./demo_Vamp --robot my_robot --benchmark
```

## Documentation

- **[Architecture Overview](ARCHITECTURE.md)**: System design and performance architecture
- **[API Reference](docs/API-Reference.md)**: Complete API documentation
- **[Contributor Guide](docs/CONTRIBUTOR-GUIDE.md)**: Development guidelines and extension patterns
- **[Planner Registry](core/OMPLPlannerRegistry.h)**: Extensible planner registration system with built-in support for common planners
- **[Visual Diagrams](docs/diagrams/)**: Architecture diagrams and system flows

## Performance Notes

**Measured Improvements:**
- 8x collision detection speedup through SIMD vectorization
- Zero-allocation hot paths via function-local static buffers
- Cache-optimized memory access with Structure-of-Arrays layout

**Dependencies:**
- Robot complexity (joint count, collision spheres)
- Environment density (obstacle count and distribution)  
- Hardware SIMD capabilities (AVX2 recommended)