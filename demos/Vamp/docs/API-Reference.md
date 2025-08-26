# VAMP API Reference Guide

## Overview
This document provides comprehensive API documentation for the VAMP motion planning system, focusing on public interfaces and extension points for developers.

## Core APIs

### VampOMPLPlanner<Robot>

The main facade interface for VAMP motion planning integration.

#### Constructor
```cpp
VampOMPLPlanner(std::unique_ptr<RobotConfig<Robot>> robotConfiguration,
               std::unique_ptr<EnvironmentFactory> environmentFactory)
```
**Parameters:**
- `robotConfiguration`: Robot-specific configuration (joint limits, default poses)
- `environmentFactory`: Factory for creating collision environments

**Throws:** `VampConfigurationError` if parameters are null

#### Core Methods

##### initialize()
```cpp
void initialize()
```
**Purpose:** Complete two-phase initialization (environment creation, OMPL setup)  
**Throws:** `VampConfigurationError` on initialization failure  
**Preconditions:** Object constructed with valid parameters  
**Postconditions:** Planner ready for planning operations

##### plan()
```cpp
PlanningResult plan(const PlanningConfig& planningConfiguration = PlanningConfig(),
                   const std::array<float, robotDimension>& customStartConfiguration = {},
                   const std::array<float, robotDimension>& customGoalConfiguration = {})
```
**Purpose:** Execute motion planning with optional custom start/goal  
**Returns:** Complete planning results including timing and path information  
**Throws:** `VampConfigurationError` if not initialized  

**Parameters:**
- `planningConfiguration`: Planner settings (time, planner type, parameters)
- `customStartConfiguration`: Optional override for start configuration
- `customGoalConfiguration`: Optional override for goal configuration

#### Utility Methods

##### getSpaceInformation()
```cpp
std::shared_ptr<ompl::base::SpaceInformation> getSpaceInformation() const
```
**Purpose:** Access underlying OMPL space information for advanced use cases  
**Returns:** Configured OMPL space information  
**Use Case:** Benchmarking systems requiring direct OMPL access

##### printConfiguration()
```cpp
void printConfiguration() const
```
**Purpose:** Print human-readable configuration summary for debugging

##### isInitialized()
```cpp
bool isInitialized() const
```
**Purpose:** Check initialization status  
**Returns:** true if initialize() completed successfully

### RobotRegistry

Singleton registry for robot type management with runtime string-based selection.

#### Core Methods

##### getInstance()
```cpp
static RobotRegistry& getInstance()
```
**Purpose:** Access singleton registry instance  
**Thread Safety:** Safe for concurrent access after static initialization

##### createRobotConfig()
```cpp
std::any createRobotConfig(const std::string& robotName,
                          const std::vector<float>& startConfig,
                          const std::vector<float>& goalConfig)
```
**Purpose:** Create robot configuration from string identifier  
**Returns:** Type-erased robot configuration  
**Throws:** `VampConfigurationError` for unknown robots or invalid configurations

##### createPlanner()
```cpp
std::any createPlanner(const std::string& robotName,
                      std::any robotConfig,
                      std::unique_ptr<EnvironmentFactory> envFactory)
```
**Purpose:** Create planner instance for specified robot type  
**Returns:** Type-erased planner instance

##### getRegisteredRobots()
```cpp
std::vector<std::string> getRegisteredRobots() const
```
**Purpose:** Get list of available robot names  
**Returns:** Vector of registered robot identifiers

#### Registration Methods

##### registerRobot<Robot>()
```cpp
template<typename Robot>
static void registerRobot(const std::string& name)
```
**Purpose:** Register custom robot type  
**Template Parameter:** Robot type implementing VAMP robot interface  
**Usage:** Typically called at static initialization time

### PlannerRegistry

Singleton registry for OMPL planner management with runtime string-based selection and parameter configuration.

#### Core Methods

##### getInstance()
```cpp
static PlannerRegistry& getInstance()
```
**Purpose:** Access singleton registry instance  
**Thread Safety:** Safe for concurrent access after static initialization

##### createPlanner()
```cpp
ob::PlannerPtr createPlanner(const std::string& name,
                            const ob::SpaceInformationPtr& si,
                            const std::map<std::string, std::string>& parameters = {})
```
**Purpose:** Create planner instance with parameters  
**Returns:** Configured OMPL planner instance  
**Throws:** `VampConfigurationError` for unknown planners

**Parameters:**
- `name`: Planner identifier (e.g., "RRT-Connect", "BIT*", "PRM")
- `si`: OMPL space information
- `parameters`: Parameter map for planner configuration

##### getRegisteredPlanners()
```cpp
std::vector<std::string> getRegisteredPlanners() const
```
**Purpose:** Get list of available planner names  
**Returns:** Vector of registered planner identifiers

##### isPlannerRegistered()
```cpp
bool isPlannerRegistered(const std::string& name) const
```
**Purpose:** Check if planner is available  
**Returns:** true if planner is registered

#### Registration Methods

##### registerPlanner()
```cpp
void registerPlanner(const std::string& name, PlannerAllocatorFunction allocator)
```
**Purpose:** Register custom planner with factory function  
**Parameters:**
- `name`: Unique planner identifier
- `allocator`: Factory function that creates and configures the planner

**Example:**
```cpp
PlannerRegistry::getInstance().registerPlanner("MyRRT*", 
    [](const ob::SpaceInformationPtr& si, const auto& params) {
        auto planner = std::make_shared<og::RRTstar>(si);
        // Parameters are automatically applied via OMPL's ParamSet
        return planner;
    });
```

#### Built-in Planners

The registry automatically registers three core planners:
- **"RRT-Connect"**: Fast bidirectional tree planner
- **"BIT*"**: Batch Informed Trees - asymptotically optimal
- **"PRM"**: Probabilistic Roadmap - good for complex environments

### Configuration System

#### PlanningConfig
```cpp
struct PlanningConfig {
    double planning_time = 1.0;
    double simplification_time = 0.5;
    std::string planner_name = "BIT*";
    bool write_path = false;
    std::map<std::string, std::string> planner_parameters;
};
```

#### PlanningResult
```cpp
struct PlanningResult {
    bool success = false;
    double planning_time_us = 0.0;
    double simplification_time_us = 0.0;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    size_t path_length = 0;
    std::string error_message;
    ompl::base::PathPtr solution_path;
    std::string solution_file_path;
};
```

## Extension APIs

### Custom Robot Implementation

#### Robot Interface Requirements
```cpp
struct CustomRobot {
    // Required compile-time constants
    static constexpr auto name = "custom_robot";
    static constexpr auto dimension = 7;        // Joint count
    static constexpr auto n_spheres = 10;       // Collision spheres
    static constexpr auto resolution = 64;      // Motion validation resolution
    
    // Joint limit arrays
    static constexpr std::array<float, dimension> s_a = {/* lower limits */};
    static constexpr std::array<float, dimension> s_m = {/* ranges */};
    
    // Required type definitions
    using Configuration = vamp::VectorXs<dimension>;
    
    // Required methods
    template<std::size_t rake>
    inline static auto fkcc(
        const vamp::collision::Environment<vamp::FloatVector<rake>>& environment,
        const vamp::ConfigurationBlock<rake>& q) noexcept -> bool;
    
    inline static void scale_configuration(Configuration& q) noexcept;
    inline static void descale_configuration(Configuration& q) noexcept;
};
```

#### Registration Macro
```cpp
REGISTER_VAMP_ROBOT(CustomRobot, "custom_robot");
```

### Custom Planner Registration

#### Factory Function Pattern
```cpp
registerPlanner("CustomPlanner",
    [](const ompl::base::SpaceInformationPtr& si, const auto& params) {
        auto planner = std::make_shared<CustomOMPLPlanner>(si);
        // Parameters are automatically applied via OMPL's ParamSet system
        // Custom parameter handling can be added here if needed
        return planner;
    });
```

#### Using Convenience Functions
```cpp
// Global convenience function
registerPlanner("MyPlanner", [](const auto& si, const auto& params) {
    return std::make_shared<MyPlanner>(si);
});

// Create planner by name
auto planner = createPlannerByName("MyPlanner", spaceInfo, {{"range", "0.5"}});
```

### Custom Environment Factory

#### Interface Implementation
```cpp
class CustomEnvironmentFactory : public EnvironmentFactory {
public:
    vamp::collision::Environment<float> createEnvironment() override {
        vamp::collision::Environment<float> env;
        // Add custom obstacles
        env.spheres.push_back({{x, y, z}, radius});
        return env;
    }
    
    std::string getEnvironmentName() const override {
        return "custom_environment";
    }
    
    std::string getDescription() const override {
        return "Custom environment with specialized obstacles";
    }
};
```

## Performance Guidelines

### SIMD Optimization
- **Robot Dimension**: Keep ≤ 16 DOF for optimal buffer usage
- **Resolution Parameter**: Balance validation thoroughness vs performance
- **Memory Alignment**: VAMP handles alignment automatically

### Thread Safety
- **Construction**: Thread-safe
- **Initialization**: Single-threaded only
- **Planning Operations**: Thread-safe after initialization
- **Registry Access**: Thread-safe for read operations

### Memory Management
- **RAII**: All resources managed automatically
- **Function-local Static Buffers**: Used internally for zero-allocation collision checking
- **Environment Caching**: Vectorized environments cached after creation

## Error Handling

### Exception Hierarchy
```cpp
VampConfigurationError          // Base configuration error
├── VampYamlError              // YAML parsing errors
└── VampValidationError        // Configuration validation errors
```

### Best Practices
1. **Catch specific exceptions** for targeted error handling
2. **Validate configurations early** before expensive operations
3. **Use isInitialized()** to check planner state before operations
4. **Check planning results** for success flag before using solution

## Usage Examples

### Basic Planning
```cpp
#include "VampOMPLDemo.h"

// Create configuration
PlanningConfiguration config;
config.robot_name = "panda";
config.start_config = {0, 0, 0, 0, 0, 0, 0};
config.goal_config = {1, 1, 1, 1, 1, 1, 1};
config.planning.planner_name = "RRT-Connect";
config.planning.planning_time = 5.0;

// Add obstacles
ObstacleConfig sphere{"sphere", {0.5f, 0.5f, 0.5f}, 0.1f};
config.obstacles.push_back(sphere);

// Execute planning
auto result = executeMotionPlanning(config);
if (result.success()) {
    std::cout << "Planning succeeded! Cost: " << result.final_cost() << std::endl;
}
```

### Advanced Planning with Custom Configuration
```cpp
// Create components directly
auto robotConfig = std::make_unique<RobotConfiguration<vamp::robots::Panda>>(
    "panda", startVec, goalVec);
auto envFactory = std::make_unique<ConfigurableEnvironmentFactory>(
    "test_env", obstacles);

// Create and configure planner
auto planner = createVampOMPLPlanner<vamp::robots::Panda>(
    std::move(robotConfig), std::move(envFactory));
planner->initialize();

// Custom planning configuration
PlanningConfig config;
config.planner_name = "BIT*";
config.planning_time = 10.0;
config.planner_parameters["rewire_factor"] = "1.1";

auto result = planner->plan(config);
```

### Benchmarking
```cpp
#include "VampBenchmarkManager.h"

// Create benchmark manager
auto benchmarkManager = std::make_unique<VampBenchmarkManager<vamp::robots::Panda>>(
    std::move(robotConfig), std::move(envFactory));
benchmarkManager->initialize();

// Configure benchmark
BenchmarkConfiguration benchConfig;
benchConfig.experiment_name = "Panda Navigation Benchmark";
benchConfig.planner_names = {"RRT-Connect", "BIT*", "PRM"};
benchConfig.runs = 50;
benchConfig.timeout = 5.0;

// Execute benchmark
std::string logFile = benchmarkManager->executeBenchmark(benchConfig);
std::cout << "Results saved to: " << logFile << std::endl;
```

## Migration Guide

### From Direct OMPL Usage
1. **Replace SpaceInformation creation** with VampOMPLPlanner
2. **Replace custom validators** with VAMP's vectorized validators
3. **Use robot registry** for string-based robot selection
4. **Leverage two-phase initialization** for better error handling

### From Other Motion Planning Libraries
1. **Map robot definitions** to VAMP robot interface
2. **Convert collision environments** to VAMP format
3. **Adapt planner parameters** to OMPL naming conventions
4. **Use VAMP's unified configuration system**

## Performance Tuning

### Robot-Specific Optimization
- **Adjust resolution parameter** based on robot complexity
- **Minimize collision spheres** while maintaining accuracy
- **Optimize joint limit ranges** for better space utilization

### Environment Optimization
- **Use appropriate obstacle types** (spheres for round objects, etc.)
- **Minimize obstacle count** through scene simplification
- **Consider pointcloud optimization** for complex geometries

## Troubleshooting

### Common Issues
1. **"Robot not registered"**: Use RobotRegistry::getRegisteredRobots() to check available robots
2. **"Planner not initialized"**: Call initialize() before planning operations
3. **"Invalid configuration"**: Check joint limits and dimension matching
4. **Poor performance**: Verify SIMD alignment and robot resolution settings

### Debug Information
- Use `printConfiguration()` for planner state inspection
- Check `isInitialized()` status before operations
- Examine error messages for specific failure details
- Enable OMPL logging for detailed planner behavior
