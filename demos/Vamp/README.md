# OMPL-VAMP Integration Demo

## Prerequisites
- OMPL 1.5+, VAMP C++ library, Eigen3, yaml-cpp
- C++17 compiler (GCC 9+ or Clang 10+), CMake 3.10+
- Python 3.7+ with numpy, pybullet for visualization

## Build
```bash
mkdir build && cd build
cmake -DOMPL_HAVE_VAMP=TRUE .. && make -j8
```

## Usage
```bash
./demo_Vamp                    # Basic programmatic example
./demo_Vamp --visualize        # Basic example + visualization  
./demo_Vamp config.yaml        # YAML configuration mode
```

Options:
- `--visualize`: Enable 3D visualization
- `--help`: Show usage information
- `config.yaml`: Use YAML configuration file

## Example
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
- Multi-robot support (Panda, UR5, Fetch)

## Notes
VAMP integration must be enabled at build time with `-DOMPL_HAVE_VAMP=TRUE`.
Visualization automatically detects robot type from planning configuration. 