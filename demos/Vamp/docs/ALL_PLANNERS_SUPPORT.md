# VAMP All OMPL Geometric Planners Support

## Overview

VAMP supports **43+ OMPL geometric planners** through a comprehensive planner registry system. This eliminates the need for expert software knowledge to integrate new planners, as the system automatically handles the vast majority of OMPL geometric planners with proper configuration and error handling.

## Supported Planners

VAMP supports **43+ OMPL geometric planners** organized into the following families:

### RRT Family (14 planners)
- **RRT** - Rapidly-exploring Random Tree
- **RRT-Connect** - Bidirectional RRT
- **RRT*** - Optimal RRT
- **RRTXstatic** - RRT with static rewiring
- **RRTsharp** - RRT with sharp rewiring
- **SORRTstar** - Simple Optimal RRT*
- **TRRT** - Transition-based RRT
- **BiTRRT** - Bidirectional T-RRT
- **LBTRRT** - Lower Bound Tree RRT
- **LazyRRT** - Lazy RRT
- **InformedRRTstar** - Informed RRT*
- **STRRTstar** - Stable Task and Motion RRT*
- **pRRT** - Parallel RRT
- **LazyLBTRRT** - Lazy Lower Bound T-RRT

**Note**: TSRRT and VFRRT require special constructor arguments (TaskSpaceConfig and VectorField respectively) and need custom setup beyond the basic registry.

### PRM Family (6 planners)
- **PRM** - Probabilistic Roadmap
- **PRMstar** - Optimal PRM
- **LazyPRM** - Lazy PRM
- **LazyPRMstar** - Lazy Optimal PRM
- **SPARS** - Sparse Roadmap Spanner
- **SPARStwo** - Sparse Roadmap Spanner v2

### Informed Trees Family (5 planners)
- **BIT*** - Batch Informed Trees
- **ABITstar** - Advanced Batch Informed Trees
- **AIT*** - Anytime Informed Trees
- **EIT*** - Effort Informed Trees
- **EIRMstar** - Effort Informed Roadmap*

### KPIECE Family (3 planners)
- **KPIECE** - Kinematic Planning by Interior-Exterior Cell Exploration
- **BKPIECE** - Bidirectional KPIECE
- **LBKPIECE** - Lower Bound KPIECE

### EST Family (3 planners)
- **EST** - Expanding Space Trees
- **BiEST** - Bidirectional EST
- **ProjEST** - Projection-based EST

### SBL Family (2 planners)
- **SBL** - Single-query Bidirectional Lazy planner
- **pSBL** - Parallel SBL

### FMT Family (2 planners)
- **FMT** - Fast Marching Tree
- **BFMT** - Bidirectional Fast Marching Tree

### RLRT Family (2 planners)
- **RLRT** - Robust Lazy RRT
- **BiRLRT** - Bidirectional Robust Lazy RRT

### Other Single-Query Planners (4 planners)
- **SST** - Stable Sparse RRT
- **STRIDE** - Stable TRRT with Informed Dynamic Exploration
- **PDST** - Path-Directed Subdivision Tree

### Advanced Manipulation Planners (2 planners)
- **XXL** - High-dimensional manipulation planner with workspace decomposition
- **TSRRT** - Task-space RRT for end-effector control

### Anytime/Multi-threaded Planners (2 planners)  
- **CForest** - Concurrent Forest of Trees
- **AnytimePathShortening** - Anytime Path Shortening

### Experience-based Planners (2 planners)
- **Lightning** - Lightning Retrieve Repair (learns from previous solutions)
- **Thunder** - Thunder Retrieve Repair (learns from previous solutions)

**Note**: Experience planners work seamlessly with default databases but can be customized programmatically for advanced users.

## Usage

### Basic Usage

```cpp
#include "VampPlannerRegistry.h"

// Create any planner by name (automatically optimized)
auto planner = vamp_ompl::createPlannerByName("RRT*", spaceInfo);
auto planner2 = vamp_ompl::createPlannerByName("BIT*", spaceInfo);
auto planner3 = vamp_ompl::createPlannerByName("PRM", spaceInfo);

// Create projection-based planners (automatically configured)
auto kpiece = vamp_ompl::createPlannerByName("KPIECE", spaceInfo);
auto sbl = vamp_ompl::createPlannerByName("SBL", spaceInfo);
```

### Benchmarking All Planners

```yaml
# config/all_planners_benchmark.yaml
benchmark:
  experiment_name: "Comprehensive Planner Comparison"
  runs: 50
  timeout: 5.0
  planners:
    - name: "RRT"
    - name: "RRT-Connect"
    - name: "RRT*"
    - name: "BIT*"
    - name: "PRM"
    - name: "KPIECE"
    - name: "Lightning"
    - name: "Thunder"
    # ... add any combination of the 45+ planners
```

### Robot-Specific Optimizations

```cpp
#include "VampRobotSpecificConfig.h"

// Get optimized parameters for a specific robot
auto pandaParams = VampRobotSpecificConfig::getOptimizedParameters("panda");
auto ur5Params = VampRobotSpecificConfig::getOptimizedParameters("ur5");

// Apply robot-specific parameters to a planner
auto planner = vamp_ompl::createPlannerByName("RRT*", spaceInfo);
VampRobotSpecificConfig::applyRobotSpecificParameters(planner.get(), "panda", "RRT*");

// Get recommended planners for a robot type
auto recommended = VampRobotSpecificConfig::getRecommendedPlanners("panda");
```

### Custom Planner Registration

```cpp
// Register a custom planner with specific parameters
vamp_ompl::registerCustomPlanner("MyCustomRRT", [](const ob::SpaceInformationPtr& si) {
    auto planner = std::make_shared<og::RRT>(si);
    planner->params().setParam("range", "0.2");
    planner->params().setParam("goal_bias", "0.1");
    return planner;
});

// Use the custom planner
auto customPlanner = vamp_ompl::createPlannerByName("MyCustomRRT", spaceInfo);
```

### Experience-based Planners

Experience planners work out-of-the-box with default databases for seamless YAML benchmarking:

```cpp
// Use experience planners directly (with default databases)
auto lightning = vamp_ompl::createPlannerByName("Lightning", spaceInfo);
auto thunder = vamp_ompl::createPlannerByName("Thunder", spaceInfo);
```

For advanced users who want custom databases:

```cpp
#include "VampExperiencePlanners.h"

// Register custom experience planners with specific databases
vamp_ompl::registerCustomPlanner("MyLightning", [](const ob::SpaceInformationPtr& si) {
    return vamp_ompl::createPersistentLightningPlanner(si, "/path/to/my/lightning.db");
});

vamp_ompl::registerCustomPlanner("MyThunder", [](const ob::SpaceInformationPtr& si) {
    return vamp_ompl::createPersistentThunderPlanner(si, "/path/to/my/thunder.db");
});

// Or create with robot/environment specific configuration
auto lightning = vamp_ompl::createLightningPlanner(si, "panda", "kitchen");
auto thunder = vamp_ompl::createThunderPlanner(si, "ur5", "assembly");
```

### Getting Available Planners

```cpp
// Get all planner names
auto allPlanners = vamp_ompl::getAllPlannerNames();

// Check if a planner is available
bool available = vamp_ompl::VampPlannerRegistry::getInstance().isPlannerAvailable("RRT*");

// Get planner count
size_t count = vamp_ompl::VampPlannerRegistry::getInstance().getPlannerCount();
```

## Architecture

### VampPlannerRegistry

The `VampPlannerRegistry` class provides:

- **Singleton Pattern**: Global registry instance
- **Factory Pattern**: Lambda-based planner creation
- **Registry Pattern**: Centralized planner management
- **Open/Closed Principle**: Extensible without modification

### Key Features

1. **Automatic Registration**: All 45 OMPL geometric planners are automatically registered
2. **Error Handling**: Comprehensive error messages with available planner suggestions
3. **Runtime Extensibility**: Custom planners can be registered at runtime
4. **Type Safety**: Strong typing with proper OMPL integration
5. **Performance**: Zero runtime overhead for planner creation

### Integration Points

- **VampBenchmarkManager**: Uses registry for comprehensive benchmarking
- **OMPLPlanningContext**: Uses registry for planner creation
- **VampOMPLPlanner**: Indirectly benefits through context integration

## Error Handling

The system provides comprehensive error handling:

```cpp
try {
    auto planner = vamp_ompl::createPlannerByName("UnknownPlanner", spaceInfo);
} catch (const vamp_ompl::VampConfigurationError& e) {
    // Error message includes available planners
    std::cerr << e.what() << std::endl;
    // Output: "Unknown planner: 'UnknownPlanner'. Available planners: RRT, RRT-Connect, RRT*, ..."
}
```

## Performance Considerations

- **Lazy Loading**: Planners are created only when needed
- **Memory Efficiency**: Shared pointers for proper resource management
- **Compile-time Optimization**: Lambda functions are optimized by the compiler
- **Zero Overhead**: No runtime cost for planner registry access

## Extending the System

### Adding New OMPL Planners

When OMPL adds new geometric planners, they can be easily integrated:

1. Add the include to `VampPlannerRegistry.h`
2. Add the registration in `registerAllOMPLPlanners()`
3. No other changes required

### Custom Planner Integration

```cpp
// Custom planner class
class MyCustomPlanner : public ompl::base::Planner {
public:
    MyCustomPlanner(const ompl::base::SpaceInformationPtr& si)
        : ompl::base::Planner(si, "MyCustomPlanner") {}
    
    // Implement required methods...
};

// Register with VAMP
vamp_ompl::registerCustomPlanner("MyCustomPlanner", 
    [](const ob::SpaceInformationPtr& si) {
        return std::make_shared<MyCustomPlanner>(si);
    });
```

## Benefits

1. **Comprehensive Coverage**: All OMPL geometric planners supported
2. **Expert Knowledge Not Required**: Automatic integration and configuration
3. **Projection Support**: Proper projection functions for projection-based planners
6. **Extensible**: Easy to add custom planners
7. **Robust**: Comprehensive error handling and validation
8. **Maintainable**: Clean architecture with clear separation of concerns

## Conclusion

VAMP provides comprehensive support for all OMPL geometric planners without requiring expert software knowledge. The system automatically handles planner integration, configuration, and error handling, making it easy to benchmark and compare different planning algorithms.
