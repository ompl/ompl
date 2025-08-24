# ADR-002: Registry Pattern Architecture

## Status
IMPLEMENTED

## Context
The system needs to support multiple robot types (Panda, UR5, Fetch, custom robots) and multiple planner types (RRT-Connect, BIT*, PRM, custom planners) while maintaining type safety and enabling runtime selection. Traditional approaches either sacrifice type safety for flexibility or require compile-time selection.

## Decision
Implement Registry patterns for both robots and planners:
1. **RobotRegistry**: Uses type erasure (`std::any`) for runtime robot selection with compile-time type safety
2. **PlannerRegistry**: Uses factory functions for runtime planner registration and creation

## Architecture

### Robot Registry Components
1. **RobotRegistry**: Singleton managing robot type registration
2. **RobotHandler**: Abstract interface for type-erased operations
3. **TypedRobotHandler<Robot>**: Concrete implementations maintaining type safety
4. **REGISTER_VAMP_ROBOT**: Macro for automatic registration

### Planner Registry Components
1. **PlannerRegistry**: Singleton managing planner factory functions
2. **PlannerAllocatorFunction**: Factory function type for planner creation
3. **registerPlanner()**: Global function for planner registration
4. **createPlannerByName()**: Global function for planner creation
5. **OMPL ParamSet**: Built-in parameter management system

### Type Safety Mechanism
```cpp
// Compile-time type safety
template<typename Robot>
class TypedRobotHandler : public RobotHandler {
    // Type-safe operations with Robot template parameter
};

// Runtime polymorphism
std::map<std::string, std::unique_ptr<RobotHandler>> handlers;
```

### Registration Strategy

#### Robot Registration
- **Built-in robots**: Auto-registered at static initialization
- **Custom robots**: Manual registration via macro
- **Runtime discovery**: Query available robots dynamically

#### Planner Registration
- **Built-in planners**: 3 core planners auto-registered (RRT-Connect, BIT*, PRM)
- **Custom planners**: Runtime registration via registerPlanner() function
- **Parameter management**: Uses OMPL's native ParamSet system for type-safe configuration
- **Extensible**: New planners added without core code modification

## Implementation Benefits

### Type Safety
- Compile-time validation of robot interface compliance
- Template-based type checking prevents configuration mismatches
- SFINAE constraints ensure interface correctness
- OMPL ParamSet provides type-safe parameter validation

### Runtime Flexibility
- String-based robot selection for configuration systems
- Dynamic robot discovery for tools and GUIs
- Plugin architecture for external robot definitions
- Runtime planner registration and parameter configuration

### Performance
- Zero-cost abstractions for type-safe operations
- Minimal overhead for type erasure (only at factory boundaries)
- Template instantiation only for actually used robot types
- OMPL's optimized parameter system for planner configuration

## Consequences

### Positive
- **Extensibility**: New robots and planners without core modifications
- **Type Safety**: Compile-time validation prevents runtime errors
- **Usability**: Simple string-based robot and planner selection
- **Performance**: Optimal code generation for robot-specific operations
- **Standards Compliance**: Uses OMPL's standard parameter management

### Negative
- Code complexity in registry implementation
- Template compilation overhead for multiple robot types
- Binary size increase proportional to registered robot count

### Neutral
- Minimal impact on existing OMPL integration
- Compatible with future robot type additions
- Transparent to most user code

## Alternatives Considered

1. **Pure template approach**: Rejected due to lack of runtime flexibility
2. **Virtual inheritance**: Rejected due to performance overhead and complexity
3. **Function pointer tables**: Rejected due to type safety concerns
4. **Preprocessor macros**: Rejected due to poor debugging experience

## Usage Patterns

### Robot Registration
```cpp
// Automatic registration for built-in robots
REGISTER_VAMP_ROBOT(vamp::robots::Panda, "panda");

// Manual registration for custom robots
RobotRegistry::getInstance().registerRobot<MyCustomRobot>("my_robot");
```

### Planner Registration
```cpp
// Register custom planner
registerPlanner("AORRTC", [](const ob::SpaceInformationPtr& si, const auto& params) {
    auto planner = std::make_shared<ompl::geometric::AORRTC>(si);
    // Apply parameters using OMPL's ParamSet
    for (const auto& [key, value] : params) {
        if (planner->params().hasParam(key)) {
            planner->params().setParam(key, value);
        }
    }
    return planner;
});
```

### Runtime Selection
```cpp
// Configuration-driven robot selection
auto& robot_registry = RobotRegistry::getInstance();
auto planner = robot_registry.createPlanner(robot_name, config, env_factory);

// Configuration-driven planner selection
auto ompl_planner = createPlannerByName(planner_name, space_info, parameters);
```

## Error Handling Strategy
- **Unknown robot**: Descriptive error with available options
- **Configuration mismatch**: Early validation with specific error messages
- **Type safety violations**: Compile-time errors with helpful diagnostics

## Future Considerations
- Plugin system for dynamically loaded robot libraries
- Metadata system for robot capabilities discovery
- Version compatibility checking for robot definitions

## References
- "Modern C++ Design" by Alexandrescu (type erasure patterns)
- "Effective C++" by Meyers (template programming best practices)
- OMPL planner registration architecture
