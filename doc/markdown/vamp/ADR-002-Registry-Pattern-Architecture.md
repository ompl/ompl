# ADR-002: Registry Pattern Architecture

## Status
**IMPLEMENTED**

## Context
The system needs to support multiple robot types (Panda, UR5, Fetch, custom robots) and multiple planner types (RRT-Connect, BIT*, PRM, custom planners) while maintaining type safety and enabling runtime selection. Traditional approaches either sacrifice type safety for flexibility or require compile-time selection.

## Decision
Implement Registry patterns with unified architecture:
1. **RobotRegistry**: Uses type erasure (`std::any`) for runtime robot selection with compile-time type safety
2. **Integrated Planner Management**: Planner registry functionality integrated into OMPLPlanningContext.

### Architecture Benefits

**Unified Design**: Single class (`OMPLPlanningContext`) handles both OMPL integration and planner management, reducing complexity and eliminating the need for separate registry classes.

**Type Safety**: Template-based robot handling ensures compile-time type checking while allowing runtime robot selection through type erasure.

**Extensibility**: Both robot and planner registries support runtime registration of custom implementations without core modifications.

**Thread Safety**: Function-local static initialization with mutex protection ensures safe concurrent access.

## Implementation

### Robot Registry Pattern
```cpp
class RobotRegistry {
    template<typename Robot>
    void registerRobot(const std::string& name);
    
    std::any createRobotConfig(const std::string& robot_name, 
                              std::vector<float> start_config, 
                              std::vector<float> goal_config);
};
```

### Integrated Planner Management
```cpp
template<typename Robot>
class OMPLPlanningContext {
    // Built-in planners: RRT-Connect, BIT*, PRM
    static std::map<std::string, PlannerAllocatorFunction>& getPlannerRegistry();
    
    // Extensibility
    static void registerPlanner(const std::string& name, PlannerAllocatorFunction allocator);
    
    // Usage
    static ob::PlannerPtr createPlannerByName(const std::string& name, 
                                             const ob::SpaceInformationPtr& si,
                                             const std::map<std::string, std::string>& parameters);
};
```

### Registration Patterns
```cpp
// Robot registration (automatic via RAII)
REGISTER_VAMP_ROBOT(vamp::robots::Panda, "panda");

// Custom planner registration
OMPLPlanningContext<Robot>::registerPlanner("MyRRT*", 
    [](const ob::SpaceInformationPtr& si, const auto& params) {
        auto planner = std::make_shared<og::RRTstar>(si);
        // Configure parameters...
        return planner;
    });
```

## Consequences

### Positive
- **Architecture**: Single unified class reduces complexity
- **Type Safety**: Compile-time checking with runtime flexibility
- **Extensibility**: Easy addition of new robots and planners
- **Thread Safety**: Safe concurrent access to registries
- **Maintainability**: Fewer files and cleaner dependencies

### Negative
- **Learning Curve**: Developers need to understand type erasure patterns
- **Template Complexity**: Heavy use of templates may increase compilation time

## Alternatives Considered

1. **Separate Registry Classes**: Would require multiple files and complex inter-dependencies
2. **Pure Template Approach**: Would lose runtime robot selection capability
3. **Factory Pattern**: Would require more boilerplate code for each robot type

## Implementation Notes

- Function-local statics ensure proper initialization order
- Type erasure (`std::any`) maintains performance while enabling flexibility
- RAII registration pattern ensures automatic setup
- Parameter handling uses OMPL's introspection system for type safety
