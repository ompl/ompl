# ADR-003: Two-Phase Initialization Pattern

## Status
TBD

## Context
Complex systems like motion planners require careful initialization ordering: environment creation, OMPL setup, validator configuration, and collision system initialization. Traditional constructor-based initialization either fails to handle errors gracefully or requires throwing exceptions from constructors.

## Decision
Implement two-phase initialization pattern separating object construction from system setup, enabling proper error handling and clear initialization semantics.

## Architecture Design

### Phase 1: Construction
- **Purpose**: Object creation with dependency injection
- **Operations**: Parameter validation, dependency storage
- **Error Handling**: Exceptions for invalid parameters
- **State**: Object exists but not operational

### Phase 2: Initialization  
- **Purpose**: System setup and resource allocation
- **Operations**: Environment creation, OMPL configuration, validator setup
- **Error Handling**: Detailed error reporting with recovery information
- **State**: Object fully operational

## Implementation Pattern

```cpp
class VampOMPLPlanner {
public:
    // Phase 1: Construction with dependency injection
    VampOMPLPlanner(std::unique_ptr<RobotConfig<Robot>> robotConfig,
                    std::unique_ptr<EnvironmentFactory> envFactory);
    
    // Phase 2: Initialization with error handling
    void initialize();
    
    // Operational methods (require initialization)
    PlanningResult plan(const PlanningConfig& config);
    
private:
    bool m_isInitialized{false};
};
```

## Initialization Sequence

1. **Environment Creation**: Scalar collision environment from factory
2. **Vectorization**: Transform to SIMD-optimized format  
3. **State Space Setup**: Configure OMPL state space with robot limits
4. **Validator Creation**: Instantiate VAMP collision validators
5. **OMPL Integration**: Connect validators to space information
6. **Problem Configuration**: Set default start/goal from robot config

## Benefits

### Error Handling
- **Graceful failure**: Clear error messages without object corruption
- **Recovery information**: Detailed diagnostics for troubleshooting
- **State consistency**: Object remains in valid state after failed initialization

### Resource Management
- **Lazy allocation**: Expensive resources created only when needed
- **Exception safety**: RAII principles maintained throughout
- **Memory efficiency**: Resources allocated only for operational planners

### Testing & Debugging
- **Unit testability**: Each phase testable in isolation
- **Clear state transitions**: Explicit operational vs non-operational states
- **Debugging support**: Initialization status queryable

## Consequences

### Positive
- **Robust error handling** without constructor exceptions
- **Clear state semantics** for debugging and testing
- **Flexible initialization** enabling advanced use cases
- **Resource efficiency** through lazy allocation

### Negative
- Additional method call required before use
- State checking overhead in operational methods
- Potential for user error (forgetting initialization)

### Neutral
- Compatible with RAII principles
- Standard pattern in complex systems
- No performance impact after initialization

## Error Handling Strategy

### Construction Errors
```cpp
// Invalid dependencies
throw VampConfigurationError("Robot configuration cannot be null");
```

### Initialization Errors
```cpp
// Environment creation failure
throw VampConfigurationError("Failed to create environment: " + details);
```

### Usage Errors
```cpp
// Method called before initialization
if (!m_isInitialized) {
    throw VampConfigurationError("Planner not initialized. Call initialize() first.");
}
```

## Usage Patterns

### Standard Usage
```cpp
auto planner = createVampOMPLPlanner(robotConfig, envFactory);
planner->initialize();  // Explicit initialization
auto result = planner->plan(config);
```

### Error Handling
```cpp
try {
    planner->initialize();
} catch (const VampConfigurationError& e) {
    // Handle initialization failure with detailed error message
    std::cerr << "Initialization failed: " << e.what() << std::endl;
}
```

## Alternatives Considered

1. **Constructor initialization**: Rejected due to exception handling complexity
2. **Factory methods**: Rejected due to loss of dependency injection benefits
3. **Lazy initialization**: Rejected due to unclear semantics and thread safety issues
4. **Builder pattern**: Considered but too complex for this use case

## Thread Safety Considerations
- **Construction**: Thread-safe (no shared state)
- **Initialization**: Not thread-safe (single-threaded expected)
- **Operations**: Thread-safe after initialization (read-only operations)

## Performance Characteristics
- **Construction**: O(1) - parameter validation only
- **Initialization**: O(n) - proportional to environment complexity
- **Operation checks**: O(1) - simple boolean check

## Future Extensions
- **Partial initialization**: Support for incremental setup
- **Re-initialization**: Support for reconfiguration without reconstruction
- **Async initialization**: Background initialization for improved responsiveness

## References
- "Effective C++" by Scott Meyers (initialization best practices)
- "C++ Core Guidelines" (constructor design)
- Gang of Four patterns (initialization patterns)
