# VAMP Contributor Guide

## Overview
This guide helps new contributors understand the VAMP architecture, design principles, and extension patterns. VAMP integrates high-performance SIMD collision detection with OMPL's motion planning algorithms while maintaining clean architectural separation.

## Architecture Principles

### Core Design Philosophy

#### 1. Performance Through Vectorization
- **SIMD-first design**: All collision detection optimized for parallel processing
- **Memory layout optimization**: Structure-of-Arrays (SOA) for cache efficiency  
- **Zero-allocation hot paths**: Function-local static buffers eliminate memory allocations
- **Compile-time optimization**: Template specialization for robot-specific code

#### 2. Clean Separation of Concerns
- **OMPL handles planning**: Algorithm implementation and path optimization
- **VAMP handles collision detection**: Vectorized environment queries
- **Clear interfaces**: Well-defined boundaries between subsystems
- **Dependency injection**: Components accept configured dependencies

#### 3. Type Safety with Runtime Flexibility
- **Compile-time validation**: Template constraints ensure interface compliance
- **Runtime selection**: String-based robot/planner selection for configuration systems
- **Type erasure**: `std::any` provides polymorphism without virtual function overhead
- **SFINAE constraints**: Template metaprogramming for robust interfaces

#### 4. Extensibility Without Modification
- **Open/Closed Principle**: Open for extension, closed for modification
- **Plugin architecture**: New robots/planners without core changes
- **Factory patterns**: Runtime creation from string identifiers
- **Registry patterns**: Automatic registration of new types

## Design Patterns Reference

### 1. Facade Pattern - VampOMPLPlanner
**Purpose:** Unified interface hiding integration complexity

```cpp
// Complex subsystem integration hidden behind simple interface
template<typename Robot>
class VampOMPLPlanner {
    // Coordinates between VAMP collision detection and OMPL planning
    // Provides single entry point for all planning operations
};
```

**Benefits:**
- Simplified client code
- Decoupled subsystem dependencies  
- Consistent error handling across integration points

### 2. Adapter Pattern - OMPLPlanningContext
**Purpose:** Bridge OMPL and VAMP interfaces

```cpp
// Adapts OMPL's interface to work with VAMP's collision detection
template<typename Robot>
class OMPLPlanningContext {
    // Translates between OMPL and VAMP configuration formats
    // Manages OMPL-specific setup while using VAMP validators
};
```

**Benefits:**
- Maintains OMPL compatibility
- Clean separation of library concerns
- Reusable across different integration scenarios

### 3. Registry Pattern - RobotRegistry
**Purpose:** Type-erased management with compile-time safety

```cpp
class RobotRegistry {
    // Maps string identifiers to type-safe robot handlers
    // Enables runtime robot selection with compile-time validation
};
```

**Benefits:**
- Runtime flexibility without sacrificing type safety
- Automatic registration of new robot types
- Plugin-compatible architecture

### 4. Factory Pattern - PlannerFactory
**Purpose:** Runtime planner creation with extensibility

```cpp
class PlannerFactory {
    // Creates OMPL planners from string identifiers
    // Supports runtime registration of new planner types
};
```

**Benefits:**
- Open/Closed Principle compliance
- Configuration-driven planner selection
- Easy addition of new planning algorithms

### 5. Template Method Pattern - Planning Workflow
**Purpose:** Standardized algorithm structure with customization points

```cpp
// Standard planning workflow:
// 1. Validate configuration
// 2. Setup environment and state space
// 3. Create and configure planner
// 4. Execute planning
// 5. Process and simplify results
// 6. Generate output
```

**Benefits:**
- Consistent behavior across robot types
- Customizable while maintaining structure
- Easy to understand and debug

## Extension Patterns

### Adding Custom Robots

#### 1. Define Robot Structure
```cpp
namespace vamp::robots {
    struct CustomRobot {
        // Required compile-time metadata
        static constexpr auto name = "custom_robot";
        static constexpr auto dimension = 7;
        static constexpr auto n_spheres = 10;
        static constexpr auto resolution = 64;
        
        // Joint limits (VAMP format)
        static constexpr std::array<float, dimension> s_a = {/* lower bounds */};
        static constexpr std::array<float, dimension> s_m = {/* ranges */};
        
        // Required type definitions
        using Configuration = vamp::VectorXs<dimension>;
        
        // Vectorized forward kinematics + collision checking
        template<std::size_t rake>
        inline static auto fkcc(
            const vamp::collision::Environment<vamp::FloatVector<rake>>& environment,
            const vamp::ConfigurationBlock<rake>& q) noexcept -> bool
        {
            // Implement vectorized forward kinematics
            // Return true if all configurations are collision-free
        }
        
        // Configuration scaling for OMPL integration
        inline static void scale_configuration(Configuration& q) noexcept;
        inline static void descale_configuration(Configuration& q) noexcept;
    };
}
```

#### 2. Register Robot Type
```cpp
// Automatic registration (preferred)
REGISTER_VAMP_ROBOT(vamp::robots::CustomRobot, "custom_robot");

// Manual registration (for dynamic scenarios)
RobotRegistry::getInstance().registerRobot<vamp::robots::CustomRobot>("custom_robot");
```

### Adding Custom Planners

#### 1. Implement OMPL Planner Interface
```cpp
namespace custom {
    class MyPlanner : public ompl::base::Planner {
    public:
        MyPlanner(const ompl::base::SpaceInformationPtr& si)
            : ompl::base::Planner(si, "MyPlanner") {}
        
        ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc) override {
            // Implement planning algorithm
            // Use si_->checkMotion() and si_->isValid() for collision checking
            // VAMP's vectorized validators will be called automatically
        }
        
        void setup() override { /* Setup planner data structures */ }
        void clear() override { /* Clear planner state */ }
    };
}
```

#### 2. Register Planner
```cpp
// Register with factory for runtime creation
OMPLPlanningContext<Robot>::registerPlanner("MyPlanner",
    [](const ompl::base::SpaceInformationPtr& si) {
        auto planner = std::make_shared<custom::MyPlanner>(si);
        // Optional: Configure planner parameters
        return planner;
    });
```

### Adding Custom Environments

#### 1. Implement Environment Factory
```cpp
class CustomEnvironmentFactory : public EnvironmentFactory {
    std::vector<ObstacleConfig> obstacles_;
    
public:
    CustomEnvironmentFactory(std::vector<ObstacleConfig> obstacles)
        : obstacles_(std::move(obstacles)) {}
    
    vamp::collision::Environment<float> createEnvironment() override {
        vamp::collision::Environment<float> env;
        
        // Convert obstacles to VAMP format
        for (const auto& obstacle : obstacles_) {
            if (obstacle.type == "sphere") {
                env.spheres.push_back({{obstacle.position[0], 
                                      obstacle.position[1], 
                                      obstacle.position[2]}, 
                                     obstacle.radius});
            }
            // Handle other obstacle types...
        }
        
        return env;
    }
    
    std::string getEnvironmentName() const override { return "custom_environment"; }
    std::string getDescription() const override { return "Custom obstacle configuration"; }
};
```

## Code Style Guidelines

### Template Programming
```cpp
// Use SFINAE for interface constraints
template<typename Robot>
class VampComponent {
    static_assert(has_fkcc_method<Robot>::value, 
                 "Robot must implement fkcc method");
    static_assert(Robot::dimension > 0, 
                 "Robot dimension must be positive");
};

// Prefer compile-time constants over runtime parameters
static constexpr std::size_t dimension = Robot::dimension;
```

### Error Handling
```cpp
// Use specific exception types
throw VampConfigurationError("Robot configuration invalid: " + details);

// Provide actionable error messages
throw VampConfigurationError("Unknown robot '" + name + "'. Available: " + 
                           getAvailableRobots());

// Validate early and fail fast
if (!config.isValid()) {
    throw VampConfigurationError("Configuration validation failed: " + 
                               config.getValidationErrors());
}
```

### Memory Management
```cpp
// Use RAII throughout
class Resource {
    std::unique_ptr<Data> data_;
public:
    Resource() : data_(std::make_unique<Data>()) {}
    // Automatic cleanup via RAII
};

// Prefer unique_ptr for ownership transfer
std::unique_ptr<EnvironmentFactory> createFactory() {
    return std::make_unique<ConfigurableEnvironmentFactory>(config);
}

// Use shared_ptr only when needed for shared ownership
std::shared_ptr<ompl::base::SpaceInformation> getSpaceInformation() const;
```

### Performance Considerations
```cpp
// Use constexpr for compile-time computation
static constexpr std::size_t bufferSize = Robot::dimension * simdWidth;

// Prefer stack allocation for small objects
std::array<float, dimension> buffer;  // vs std::vector<float>

// Use function-local static for per-instantiation resources
static std::array<float, maxDimension> conversionBuffer;
```

## Performance Optimization Guidelines

### SIMD Optimization
- **Data alignment**: Ensure SIMD-compatible memory alignment
- **Loop vectorization**: Structure loops for compiler auto-vectorization
- **Memory access patterns**: Prefer sequential access for cache efficiency
- **Instruction selection**: Use appropriate SIMD intrinsics when needed

### Memory Optimization
- **Buffer reuse**: Use function-local static buffers to avoid allocations
- **Memory layout**: Prefer SOA over AOS for vectorized operations
- **Cache optimization**: Consider data locality in algorithm design
- **Lazy initialization**: Defer expensive operations until needed

### Compilation Optimization
- **Template specialization**: Specialize for common robot types
- **Constexpr evaluation**: Move computations to compile time
- **Link-time optimization**: Enable LTO for production builds
- **Profile-guided optimization**: Use PGO for hot paths

## Custom Robot Development

### Robot Interface Requirements

All custom robots must implement the VAMP robot interface:

```cpp
namespace vamp::robots {
    struct CustomRobot {
        // Required compile-time constants
        static constexpr auto name = "custom_robot";
        static constexpr auto dimension = 7;        // Number of joints
        static constexpr auto n_spheres = 10;       // Collision spheres
        static constexpr auto resolution = 32;      // Motion validation resolution
        
        // Joint limits (scaled to [0,1])
        static constexpr std::array<float, dimension> s_a = {/* lower bounds */};
        static constexpr std::array<float, dimension> s_m = {/* ranges */};
        
        // Required methods
        template <std::size_t rake>
        inline static auto fkcc(
            const collision::Environment<FloatVector<rake>>& environment,
            const ConfigurationBlock<rake>& q) noexcept -> bool;
            
        inline static void scale_configuration(Configuration& q) noexcept;
        inline static void descale_configuration(Configuration& q) noexcept;
    };
}

// Register the robot
REGISTER_VAMP_ROBOT(vamp::robots::CustomRobot, "custom_robot");
```

### Implementation Guidelines

1. **Vectorized Forward Kinematics**: The `fkcc` method must process `rake` configurations simultaneously
2. **SIMD-Friendly Data**: Use aligned memory and avoid branching in hot paths
3. **Configuration Scaling**: Implement proper scaling/descaling for joint limits
4. **Collision Spheres**: Define sphere positions and radii for collision detection

### Testing Custom Robots

```bash
# Test basic functionality
./demo_Vamp --robot custom_robot

# Run benchmarks
./demo_Vamp --robot custom_robot --benchmark

# List all registered robots
./demo_Vamp --list-robots
```

For complete examples, see `examples/CustomRobotExample.h`.

## Debugging Guidelines

### Common Issues
1. **Template compilation errors**: Check interface compliance with static_assert
2. **Runtime registration failures**: Verify macro usage and linking
3. **Performance regression**: Profile SIMD code generation
4. **Memory issues**: Use sanitizers for leak detection

### Debugging Tools
- **Compiler explorer**: Verify SIMD code generation
- **Profilers**: Intel VTune, perf for performance analysis
- **Sanitizers**: AddressSanitizer, ThreadSanitizer for memory/threading issues
- **OMPL logging**: Enable for detailed planner behavior

### Debug Build Configuration
```cpp
// Debug assertions for development
#ifdef DEBUG
    assert(isInitialized() && "Planner not initialized");
    assert(config.isValid() && "Invalid configuration");
#endif

// Conditional compilation for debug features
#ifdef VAMP_ENABLE_DEBUG_OUTPUT
    std::cout << "Configuration: " << config.toString() << std::endl;
#endif
```

## Documentation Standards

### Code Documentation
- **API documentation**: Doxygen-style comments for all public interfaces
- **Implementation notes**: Explain complex algorithms and optimizations
- **Performance characteristics**: Document time/space complexity
- **Usage examples**: Provide clear examples for each major feature

### Architecture Documentation
- **ADRs**: Document significant architectural decisions
- **Design patterns**: Explain pattern usage and benefits
- **Extension guides**: Step-by-step instructions for common extensions
- **Performance guides**: Optimization strategies and benchmarking

## Contributing Process

### Development Workflow
1. **Fork repository** and create feature branch
2. **Implement changes** following style guidelines
3. **Add comprehensive tests** for new functionality
4. **Update documentation** including API docs and examples
5. **Run full test suite** including performance benchmarks
6. **Submit pull request** with detailed description

### Code Review Checklist
- [ ] Follows established design patterns
- [ ] Maintains type safety throughout
- [ ] Includes comprehensive error handling
- [ ] Preserves or improves performance characteristics
- [ ] Updates relevant documentation
- [ ] Follows code style guidelines

### Release Process
1. **Version compatibility** check for breaking changes
2. **Performance regression** testing against benchmarks
3. **Documentation updates** for new features
4. **Example code validation** ensure examples work correctly
5. **Integration testing** with downstream projects

## Future Architecture Considerations

### Scalability
- **Plugin system**: Dynamic loading of robot libraries
- **Distributed planning**: Multi-node planning coordination
- **GPU acceleration**: CUDA/OpenCL backend for collision detection
- **Real-time planning**: Low-latency planning for dynamic environments

### Maintainability
- **Module boundaries**: Clear separation of concerns
- **Interface stability**: Backward compatibility strategies
- **Testing infrastructure**: Automated testing and benchmarking
- **Documentation automation**: Generated docs from code

### Extensibility
- **Constraint handling**: Support for kinematic constraints
- **Dynamic environments**: Moving obstacles and changing scenes
- **Custom collision primitives**: User-defined collision shapes
- **Multi-robot planning**: Coordination between multiple robots
