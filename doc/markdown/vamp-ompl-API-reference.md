# VAMP-OMPL API Reference {#vamp-ompl-api-reference}

This document provides comprehensive API documentation for VAMP-OMPL integration.

## Quick Start

```cpp
#include "VampOmplPlanning.h"

// All VAMP-OMPL functionality is now available
vamp_ompl::PlanningConfiguration config;
config.robot_name = "panda";
config.start_config = {0, 0, 0, 0, 0, 0, 0};
config.goal_config = {1, 1, 1, 1, 1, 1, 1};
config.planning.planner_name = "RRT-Connect";

auto result = vamp_ompl::executeMotionPlanning(config);
```

## Header Organization

**Primary Header**: `VampOmplPlanning.h` - Comprehensive header providing complete VAMP-OMPL functionality

This single header includes:
- High-level planning interface (`executeMotionPlanning`)
- Robot registry system with built-in and custom robot support  
- Integrated planner management (RRT-Connect, BIT*, PRM, custom planners)
- SIMD-accelerated collision detection and motion validation
- Configuration structures and utilities
- Benchmarking infrastructure
- Visualization support

## Core Functions

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

### OMPLPlanningContext

Unified OMPL integration with integrated planner management. This class serves as the central OMPL adapter and planner registry, providing both state space management and planner creation in a single, cohesive interface.

#### Core Methods

##### setupStateSpace()

```cpp
void setupStateSpace(const RobotConfig<Robot>& robot_configuration, 
                    const VectorizedEnvironment& vectorized_environment)
```

**Purpose:** Initialize OMPL state space with robot configuration and VAMP validators  
**Parameters:**
- `robot_configuration`: Robot configuration providing joint limits
- `vectorized_environment`: VAMP collision environment for validators

##### plan()

```cpp
auto plan(const PlanningConfig& planning_configuration) -> PlanningResult
```

**Purpose:** Execute motion planning with specified configuration  
**Parameters:**
- `planning_configuration`: Planning settings including planner name, time limits, and parameters
**Returns:** Complete planning results with timing, path, and success information

#### Integrated Planner Management

The `OMPLPlanningContext` includes built-in planner management functionality:

##### Built-in Planners

- **RRT-Connect**: Fast bidirectional tree planner for quick solutions
- **BIT***: Batch Informed Trees for asymptotically optimal paths  
- **PRM**: Probabilistic Roadmap for complex environments

##### registerPlanner()

```cpp
static void registerPlanner(const std::string& name, PlannerAllocatorFunction allocator)
```

**Purpose:** Register custom planner for runtime selection  
**Parameters:**
- `name`: Unique planner identifier
- `allocator`: Factory function creating configured planner instance

**Example:**
```cpp
OMPLPlanningContext<Robot>::registerPlanner("MyRRT*", 
    [](const ob::SpaceInformationPtr& si, const auto& params) {
        auto planner = std::make_shared<og::RRTstar>(si);
        // Configure parameters using OMPL's ParamSet
        return planner;
    });
```

##### createPlannerByName()

```cpp
static ob::PlannerPtr createPlannerByName(const std::string& name,
                                         const ob::SpaceInformationPtr& si,
                                         const std::map<std::string, std::string>& parameters)
```

**Purpose:** Create planner instance by name with parameters  
**Parameters:**
- `name`: Registered planner identifier
- `si`: OMPL space information
- `parameters`: Planner-specific parameter map
**Returns:** Configured OMPL planner instance
**Throws:** `VampConfigurationError` if planner not found

##### getAvailablePlannerNames()

```cpp
static std::string getAvailablePlannerNames()
```

**Purpose:** Get comma-separated list of available planners  
**Returns:** String listing all registered planner names

#### Parameter Handling

The integrated planner management automatically applies parameters using OMPL's ParamSet introspection:

```cpp
// Parameters are automatically validated and applied
std::map<std::string, std::string> params = {
    {"range", "0.3"},
    {"intermediate_states", "false"}
};
auto planner = OMPLPlanningContext<Robot>::createPlannerByName("RRT-Connect", si, params);
```

#### Thread Safety

- **Function-Local Static Registry**: Thread-safe initialization guaranteed by C++11
- **Mutex Protection**: Write operations (registration) protected with mutex
- **Lock-Free Reads**: Planner creation doesn't require synchronization

### VampOMPLPlanner

Main planning facade that coordinates VAMP-OMPL integration.

#### Constructor

```cpp
VampOMPLPlanner(std::unique_ptr<RobotConfig<Robot>> robot_configuration,
                std::unique_ptr<EnvironmentFactory> environment_factory)
```

**Purpose:** Create planner with dependency injection  
**Parameters:**
- `robot_configuration`: Robot configuration providing limits and poses
- `environment_factory`: Factory for creating collision environments

#### initialize()

```cpp
void initialize()
```

**Purpose:** Two-phase initialization - create environment and setup OMPL  
**Note:** Must be called before planning

#### plan()

```cpp
auto plan(const PlanningConfig& planning_configuration,
         const std::array<float, robot_dimension_>& custom_start_configuration = {},
         const std::array<float, robot_dimension_>& custom_goal_configuration = {}) -> PlanningResult
```

**Purpose:** Execute motion planning  
**Parameters:**
- `planning_configuration`: Planning settings and planner selection
- `custom_start_configuration`: Optional custom start (uses robot default if empty)
- `custom_goal_configuration`: Optional custom goal (uses robot default if empty)
**Returns:** Complete planning results

### RobotRegistry

Singleton registry for type-safe robot management.

#### getInstance()

```cpp
static RobotRegistry& getInstance()
```

**Purpose:** Get singleton registry instance  
**Returns:** Reference to global robot registry

#### registerRobot()

```cpp
template<typename Robot>
void registerRobot(const std::string& name)
```

**Purpose:** Register robot type for runtime selection  
**Template Parameter:** VAMP robot type
**Parameters:**
- `name`: Unique robot identifier

#### createRobotConfig()

```cpp
std::any createRobotConfig(const std::string& robot_name,
                          std::vector<float> start_config,
                          std::vector<float> goal_config)
```

**Purpose:** Create type-erased robot configuration  
**Parameters:**
- `robot_name`: Registered robot identifier
- `start_config`: Start joint configuration
- `goal_config`: Goal joint configuration
**Returns:** Type-erased robot configuration
**Throws:** `VampConfigurationError` for invalid robot or configuration

#### Built-in Robots

- **panda**: Franka Emika Panda 7-DOF manipulator
- **ur5**: Universal Robots UR5 6-DOF arm
- **fetch**: Fetch Robotics 8-DOF mobile manipulator

### Configuration Structures

#### PlanningConfig

```cpp
struct PlanningConfig {
    double planning_time = 1.0;
    double simplification_time = 0.5;
    bool optimize_path = false;
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
    ob::PathPtr solution_path;
    std::string solution_file_path;
};
```

### Error Handling

#### VampConfigurationError

```cpp
class VampConfigurationError : public std::runtime_error
```

**Purpose:** Configuration and validation errors  
**Usage:** Thrown for invalid robot configurations, unknown planners, etc.

### Extension Patterns

#### Custom Robot Registration

```cpp
// Define robot structure
namespace vamp::robots {
    struct MyRobot {
        static constexpr auto name = "my_robot";
        static constexpr auto dimension = 6;
        // ... implement required interface
    };
}

// Register robot
REGISTER_VAMP_ROBOT(vamp::robots::MyRobot, "my_robot");
```

#### Custom Planner Registration

```cpp
// Register at runtime
OMPLPlanningContext<Robot>::registerPlanner("MyPlanner",
    [](const ob::SpaceInformationPtr& si, const auto& params) {
        auto planner = std::make_shared<MyPlanner>(si);
        // Apply parameters using OMPL ParamSet
        for (const auto& [key, value] : params) {
            if (planner->params().hasParam(key)) {
                planner->params().setParam(key, value);
            }
        }
        return planner;
    });
```

This API provides a comprehensive, type-safe, and extensible interface for VAMP-OMPL motion planning with integrated planner management.
