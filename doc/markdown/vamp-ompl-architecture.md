# VAMP-OMPL Architecture {#vamp-ompl-architecture}

This document provides architectural documentation for VAMP-OMPL, which integrates high-performance SIMD collision detection with OMPL's motion planning algorithms.

[TOC]

## Architecture Overview

### System Layers

**System Architecture Overview**: ![System Architecture](images/vamp-ompl/01-system-architecture-overview.svg)

The VAMP architecture consists of multiple layers:

1. **User Application Layer**: YAML configuration and programmatic APIs
2. **Unified Interface Layer**: Registry-based execution with `executeMotionPlanning()`
3. **Planning Facade Layer**: Main coordination components implementing design patterns
4. **OMPL Integration Layer**: Standard OMPL interfaces with VAMP bridges
5. **VAMP Performance Core**: SIMD-optimized collision detection and vectorization
6. **Robot Type System**: Built-in and extensible robot implementations
7. **Benchmarking System**: OMPL-compliant performance measurement infrastructure

### Key Components

#### 1. **VampOMPLPlanner** (Facade Pattern)

- **Purpose**: Main entry point that coordinates between VAMP and OMPL systems
- **Key Features**: 
  - Dependency injection for robot configuration and environment factory
  - Two-phase initialization (constructor + initialize())
  - Unified planning interface supporting both default and custom configurations
  - Automatic path writing and visualization integration

#### 2. **VampStateValidator & VampMotionValidator** (SIMD-Accelerated)

- **Purpose**: Bridge OMPL's validation interface with VAMP's vectorized collision detection
- **Performance Features**:
  - SIMD vectorization: Process 8 configurations simultaneously  
  - "Rake" motion validation: Spatially distributed parallel sampling
  - Function-local static buffer pools: Allocation-free hot paths
  - Zero-copy OMPL-to-VAMP configuration conversion

#### 3. **RobotRegistry** (Singleton + Registry Pattern)

- **Purpose**: Type-erased robot management with compile-time safety
- **Features**:
  - Automatic registration of built-in robots (Panda, UR5, Fetch)
  - Runtime robot creation from string identifiers
  - Thread-safe robot handler management
  - Extensible to custom robots without core modifications

---

## Performance Architecture

### SIMD Vectorization Pipeline

**SIMD Performance Architecture**: ![SIMD Performance](images/vamp-ompl/04-simd-performance-architecture.svg)

### Performance Optimizations

#### 1. **SIMD Vectorization**

- **8x Parallel Processing**: Process 8 robot configurations simultaneously using SIMD instructions
- **Structure-of-Arrays (SOA)**: Memory layout optimized for vectorized operations
- **Cache Efficiency**: Spatial locality in collision geometry access

#### 2. **"Rake" Motion Validation**

- **Parallel Sampling**: Distribute temporal samples across SIMD lanes
- **Resolution Control**: Configurable sampling density per robot type
- **Efficient Pipeline**: Better CPU instruction pipeline utilization

#### 3. **Memory Management**

- **Function-local Static Buffers**: Avoid allocations in collision checking hot path
- **Zero-copy Conversion**: Direct OMPL-to-VAMP configuration mapping
- **SIMD-aligned Memory**: Proper alignment for vectorized operations

---

## Class Architecture

### Design Patterns & Relationships

**Class Architecture & Design Patterns**: ![Class Architecture](images/vamp-ompl/02-class-architecture-patterns.svg)

The class architecture demonstrates sophisticated use of design patterns:

- **OMPL Foundation**: Standard interfaces (`StateValidityChecker`, `MotionValidator`, `Planner`)
- **VAMP Integration**: Template-based bridges with SIMD acceleration
- **Configuration System**: Hierarchical robot and environment configuration
- **Registry System**: Type-erased management with compile-time safety
- **Factory System**: Runtime creation with extensibility
- **Template Relationships**: Type-safe robot-specific implementations

---

## Planning Workflow

### Request Lifecycle

**Planning Request Sequence**: ![Planning Sequence](images/vamp-ompl/03-planning-request-sequence.svg)

The planning workflow follows a well-defined sequence:

1. **Configuration Phase**: Validation, registry lookup, planner creation
2. **Initialization Phase**: Environment setup, OMPL configuration, validator setup
3. **Planning Execution**: Planner creation, main planning loop with SIMD collision detection
4. **Solution Processing**: Path simplification, result formatting, optional file writing

**Key Performance Points**:

- Zero-copy state conversion using function-local static buffers
- SIMD "rake" sampling for motion validation (8x parallel collision checks)
- Structure-of-Arrays memory layout for cache optimization

---

## Extension Points

### Adding Custom Robots

The extension points are clearly documented in the system architecture diagram above. Key extension areas include:

#### 1. **Custom Robot Implementation**

```cpp
// Example: Define a custom robot
namespace vamp::robots {
    struct MyCustomRobot {
        static constexpr auto name = "my_robot";
        static constexpr auto dimension = 7;      // Number of joints
        static constexpr auto n_spheres = 12;     // Collision spheres
        static constexpr auto resolution = 32;    // Motion validation resolution
        
        // Joint limits
        static constexpr std::array<float, dimension> s_a = {/* lower limits */};
        static constexpr std::array<float, dimension> s_m = {/* ranges */};
        
        // Vectorized forward kinematics + collision checking
        template <std::size_t rake>
        inline static auto fkcc(
            const collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &q) noexcept -> bool
        {
            // Implement vectorized forward kinematics
            // Return true if collision-free, false otherwise
        }
        
        // Configuration scaling methods
        inline static void scale_configuration(Configuration &q) noexcept;
        inline static void descale_configuration(Configuration &q) noexcept;
    };
}

// Register the robot
REGISTER_VAMP_ROBOT(vamp::robots::MyCustomRobot, "my_robot");
```

#### 2. **Custom Planner Registration**

```cpp
// Register a custom OMPL planner
OMPLPlanningContext<Robot>::registerPlanner("MyPlanner", 
    [](const ob::SpaceInformationPtr& si) {
        auto planner = std::make_shared<MyCustomPlanner>(si);
        // Set planner-specific parameters
        return planner;
    });
```

---

## Design Patterns

### Core Patterns Used

#### 1. **Facade Pattern** (`VampOMPLPlanner`)

- **Purpose**: Provides a unified interface hiding complexity of VAMP-OMPL integration
- **Benefits**: Single point of entry, simplified client code, decoupled subsystems

#### 2. **Adapter Pattern** (`OMPLPlanningContext`)

- **Purpose**: Bridges OMPL's interface with VAMP's collision detection system
- **Benefits**: Maintains compatibility, clean separation of concerns

#### 3. **Factory Pattern** (`PlannerFactory`, `RobotRegistry`)

- **Purpose**: Runtime creation of planners and robots from string identifiers
- **Benefits**: Open/Closed Principle compliance, extensibility without modification

#### 4. **Registry Pattern** (`RobotRegistry`)

- **Purpose**: Type-erased management of different robot types
- **Benefits**: Runtime polymorphism with compile-time safety

#### 5. **Template Method Pattern** (Planning workflow)

- **Purpose**: Standardizes planning algorithm while allowing customization
- **Benefits**: Consistent interface, flexible implementation
