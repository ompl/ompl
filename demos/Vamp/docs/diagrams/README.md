# VAMP Architecture Diagrams

This directory contains comprehensive architecture diagrams for the VAMP (Vectorized Accelerated Motion Planner) system. These diagrams provide visual documentation of the system's design patterns, component relationships, and performance characteristics.

## Diagram Overview

### 1. System Architecture Overview 
**File:** `01-system-architecture-overview.svg`  
**Source:** `01-system-architecture-overview.mmd`

**Purpose:** Complete system view showing all layers and components  
**Audience:** System architects, new contributors, project managers  
**Key Elements:**
- User interface layer (YAML/API)
- Unified interface layer (Registry-based execution)
- Planning facade layer (Main coordination components)
- OMPL integration layer (Standard planning interfaces)
- VAMP performance core (SIMD collision detection)
- Robot type system (Built-in and custom robots)
- Benchmarking system (OMPL-compliant infrastructure)
- Extension points (Robot, planner, environment extensions)

**Design Patterns Highlighted:** Facade, Adapter, Registry, Factory, Strategy

### 2. Class Architecture & Design Patterns 
**File:** `02-class-architecture-patterns.svg`  
**Source:** `02-class-architecture-patterns.mmd`

**Purpose:** Detailed class relationships with UML-style documentation  
**Audience:** Software developers, code reviewers, maintainers  
**Key Elements:**
- OMPL foundation interfaces (StateValidityChecker, MotionValidator, Planner)
- VAMP core integration classes (Validators, Planners, Context)
- Configuration system hierarchy (RobotConfig, EnvironmentFactory)
- Registry system (Singleton pattern with type erasure)
- Factory system (Runtime planner creation)
- Template relationships and inheritance hierarchies

**Design Patterns Highlighted:** Template Method, Singleton, Factory Method, Adapter, Registry

### 3. Planning Request Sequence 
**File:** `03-planning-request-sequence.svg`  
**Source:** `03-planning-request-sequence.mmd`

**Purpose:** Complete lifecycle from user request to results  
**Audience:** Developers, debuggers, performance analysts  
**Key Elements:**
- Configuration validation and registry lookup
- Two-phase initialization (constructor + initialize())
- OMPL setup with VAMP validators
- Planning execution with SIMD collision detection
- Result processing and optional file output

**Performance Highlights:** Zero-copy state conversion, SIMD "rake" sampling, Structure-of-Arrays layout

### 4. SIMD Performance Architecture 
**File:** `04-simd-performance-architecture.svg`  
**Source:** `04-simd-performance-architecture.mmd`

**Purpose:** Detailed view of SIMD optimization strategies  
**Audience:** Performance engineers, VAMP core developers  
**Key Elements:**
- 8x parallel collision checking pipeline
- Structure-of-Arrays memory layout
- Function-local static buffer management
- Cache-optimized memory access patterns
- Vectorized forward kinematics implementation

**Performance Features:** 8x speedup, zero-allocation hot paths, cache efficiency

## Generating Diagrams

To regenerate SVG files from Mermaid source:

```bash
cd docs/
python3 generate_diagrams.py
```

**Requirements:**
- Python 3.6+
- Mermaid CLI (`npm install -g @mermaid-js/mermaid-cli`)

## Usage Guidelines

### For Documentation
- Reference diagrams using relative paths: `![Description](diagrams/filename.svg)`
- Always include both purpose and audience in documentation
- Link to source `.mmd` files for editing

### For Presentations
- SVG files are vector graphics suitable for any resolution
- Use PNG exports for compatibility with older presentation software
- Include diagram source attribution in presentations

### For Development
- Update `.mmd` source files when architecture changes
- Regenerate SVGs after any source modifications
- Validate diagram accuracy against actual implementation

## Maintenance

- **Review Frequency:** Update diagrams with major architectural changes
- **Validation:** Ensure diagrams match actual implementation
- **Consistency:** Maintain consistent styling and terminology across all diagrams
- **Accessibility:** Include alt-text and descriptions for screen readers 