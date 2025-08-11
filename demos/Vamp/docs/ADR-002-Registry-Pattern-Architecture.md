# ADR-002: Registry Pattern for Robot Type Management

## Status
TBD

## Context
The system needs to support multiple robot types (Panda, UR5, Fetch, custom robots) while maintaining type safety and enabling runtime robot selection. Traditional approaches either sacrifice type safety for flexibility or require compile-time robot selection.

## Decision
Implement a Registry pattern with type erasure (`std::any`) that provides runtime polymorphism while preserving compile-time type safety for robot-specific operations.

## Architecture

### Core Components
1. **RobotRegistry**: Singleton managing robot type registration
2. **RobotHandler**: Abstract interface for type-erased operations
3. **TypedRobotHandler<Robot>**: Concrete implementations maintaining type safety
4. **REGISTER_VAMP_ROBOT**: Macro for automatic registration

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
- **Built-in robots**: Auto-registered at static initialization
- **Custom robots**: Manual registration via macro
- **Runtime discovery**: Query available robots dynamically

## Implementation Benefits

### Type Safety
- Compile-time validation of robot interface compliance
- Template-based type checking prevents configuration mismatches
- SFINAE constraints ensure interface correctness

### Runtime Flexibility
- String-based robot selection for configuration systems
- Dynamic robot discovery for tools and GUIs
- Plugin architecture for external robot definitions

### Performance
- Zero-cost abstractions for type-safe operations
- Minimal overhead for type erasure (only at factory boundaries)
- Template instantiation only for actually used robot types

## Consequences

### Positive
- **Extensibility**: New robots without core modifications
- **Type Safety**: Compile-time validation prevents runtime errors
- **Usability**: Simple string-based robot selection
- **Performance**: Optimal code generation for robot-specific operations

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

### Runtime Robot Selection
```cpp
// Configuration-driven robot selection
auto& registry = RobotRegistry::getInstance();
auto planner = registry.createPlanner(robot_name, config, env_factory);
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
