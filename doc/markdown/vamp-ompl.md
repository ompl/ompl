# VAMP-OMPL Integration Demo {#vamp-ompl}

A high-performance motion planning system that integrates VAMP's SIMD-accelerated collision detection with OMPL's comprehensive planner library.

## Quick Start

### Prerequisites

- **C++ Compiler**: GCC 9+ or Clang 10+ with C++17 support
- **OMPL**: Open Motion Planning Library
- **VAMP**: Vectorized Autonomous Motion Planner
- **YAML-CPP**: For configuration file parsing
- **PyBullet** (optional, for visualization): `pip install pybullet`

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

### Basic YAML Configuration

```yaml
robot:
  name: "panda"
  description: "Franka Emika Panda arm"

planner:
  name: "RRT-Connect"
  planning_time: 5.0
  simplification_time: 1.0
  parameters:
    range: "0.3"
    intermediate_states: "false"

start_config: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
goal_config: [2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785]

obstacles:
  - type: "sphere"
    position: [0.5, 0.5, 0.5]
    radius: 0.1
  - type: pointcloud
    pointcloud_file: "environment.xyz"
    point_radius: 0.0025

# Visualization settings
visualization:
  urdf_path: "../../"
  base_position: [0.0, 0.0, 0.0]
  use_fixed_base: true

```
### Benchmarking Configuration

```yaml
robot:
  name: "panda"
  description: "Franka Emika Panda arm"

# Multiple planners for comparison
planners:
  - name: "RRT-Connect"
    parameters:
      range: "0.2"
  - name: "BIT*"
    parameters:
      rewire_factor: "1.0"
  - name: "PRM"
    parameters:
      max_nearest_neighbors: "10"

benchmark:
  experiment_name: "Fetch Navigation Benchmark"
  runs: 50
  timeout: 5.0
  memory_limit: 4096.0

start_config: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_config: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

obstacles:
  - type: sphere
    position: [0.5, 0.5, 1.0]
    radius: 0.2
```

### Benchmarking Workflow

```bash
# 1. Run benchmark
./demo_Vamp panda_benchmark.yaml --benchmark

# 2. Generate database (from OMPL root directory)
python3 scripts/ompl_benchmark_statistics.py vamp_benchmark_*.log -d results.db

# 3. Visualize at http://plannerarena.org (upload results.db)
```

### Examples

#### planar_arm_2dof (2-DOF)
```bash
# Quick planning test
./demo_Vamp --robot planar_arm_2dof

# Custom configuration
./demo_Vamp config/planar_arm_2dof_demo.yaml --visualize
```

## Custom Robot Development

### Prerequisites for Custom Robots

**⚠️ Important**: Custom robots for VAMP require specialized code generation tools:

#### 1. **Foam Tool** (Required for Spherization)
You need a spherical decomposition of your robot's collision geometry. This can be generated automatically using the [foam tool](https://github.com/CoMMALab/foam).

- **Repository**: https://github.com/CoMMALab/foam  
- **Purpose**: Generates spherical approximations of robot collision geometry from URDF
- **Output**: Sphere positions and radii for collision detection

**⚠️ Warning**: Some tuning of the robot spherization may be necessary. Start with a finer approximation and work up from there.

#### 2. **Cricket Compiler** (Required)
VAMP uses a tracing compilation step to generate optimized SIMD code for collision checking. You **must** use the [cricket compiler](https://github.com/CoMMALab/cricket) to generate the vectorized forward kinematics and collision checking methods (`fkcc` function).

- **Repository**: https://github.com/CoMMALab/cricket
- **Purpose**: Generates SIMD-optimized collision detection code from robot URDF
- **Output**: Vectorized `fkcc` method required by VAMP robot interface

### Development Workflow

1. **Prepare Robot Model**: Ensure you have a valid URDF with collision geometry
2. **Generate Spherization**: Use `foam` to create spherical collision approximation
3. **Generate SIMD Code**: Use `cricket` compiler to generate vectorized collision checking
4. **Implement Robot Interface**: Create robot struct with required VAMP interface
5. **Register Robot**: Use `REGISTER_VAMP_ROBOT` macro for integration
6. **Test and Validate**: Verify collision detection and planning functionality

### Quick Custom Robot Example

```cpp
// 1. Define robot (see examples/CustomRobotExample.h)
namespace vamp::robots {
    struct MyRobot {
        static constexpr auto name = "my_robot";
        static constexpr auto dimension = 6;
        static constexpr auto n_spheres = 8;
        static constexpr auto resolution = 64;
        
        // Joint limits (replace with actual values)
        static constexpr std::array<float, 6> s_a = {-3.14, -1.57, -3.14, -3.14, -3.14, -3.14};
        static constexpr std::array<float, 6> s_m = {6.28, 3.14, 6.28, 6.28, 6.28, 6.28};
        
        // SIMD collision checking (generated by cricket)
        template<std::size_t rake>
        inline static auto fkcc(
            const vamp::collision::Environment<vamp::FloatVector<rake>>& environment,
            const vamp::ConfigurationBlock<rake>& q) noexcept -> bool {
            // Generated by cricket compiler
        }
        
        // Configuration scaling
        inline static void scale_configuration(Configuration& q) noexcept {
            for (size_t i = 0; i < dimension; ++i) {
                q[i] = (q[i] - s_a[i]) / s_m[i];
            }
        }
        
        inline static void descale_configuration(Configuration& q) noexcept {
            for (size_t i = 0; i < dimension; ++i) {
                q[i] = q[i] * s_m[i] + s_a[i];
            }
        }
    };
}

// 2. Register robot
REGISTER_VAMP_ROBOT(vamp::robots::MyRobot, "my_robot");

// 3. Use in YAML or command line
./demo_Vamp --robot my_robot --benchmark
```

**Note**: The critical `fkcc` method must be generated using the cricket compiler - manual implementation is not recommended due to the complexity of SIMD vectorization.

## Documentation

- **[Architecture Overview](vamp-ompl-architecture.md)**: System design and performance architecture
- **[API Reference](vamp-ompl-API-reference.md)**: Complete API documentation

## Examples and Demos

### Configuration Files
- `config/panda_demo.yaml`: Basic Panda planning example
- `config/panda_benchmark.yaml`: Comprehensive benchmarking setup
- `config/planar_arm_demo.yaml`: 2DOF planar arm example

### Source Code Examples
- `examples/CustomRobotExample.h`: Template for custom robot development
- `demos/VampDemo.cpp`: Main demonstration application
- `benchmarking/VampBenchmarkManager.h`: Benchmarking infrastructure

This comprehensive guide provides everything needed to get started with VAMP-OMPL motion planning, from basic usage to advanced custom robot development.
