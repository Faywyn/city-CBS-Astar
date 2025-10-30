# Coding Standards for city-CBS-Astar

This document outlines the coding standards and best practices for the city-CBS-Astar project.

## Table of Contents
1. [General Principles](#general-principles)
2. [Code Style](#code-style)
3. [Naming Conventions](#naming-conventions)
4. [Documentation](#documentation)
5. [Error Handling](#error-handling)
6. [Performance Considerations](#performance-considerations)

## General Principles

### Keep It Simple
- Write clear, readable code over clever solutions
- Avoid unnecessary complexity
- Each function should have a single, well-defined purpose

### Use Constants
- All magic numbers and strings should be defined as named constants in `config.h`
- Constants should have descriptive names that explain their purpose
- Use `constexpr` for compile-time constants

### Const Correctness
- Use `const` for function parameters that won't be modified
- Use `const` for member functions that don't modify object state
- Prefer `const` references over pass-by-value for non-primitive types

## Code Style

### Formatting
- Use the provided `.clang-format` configuration
- Indentation: 2 spaces (no tabs)
- Line length: Maximum 120 characters
- Brace style: LLVM style (opening brace on same line)

### File Organization
- One class per header/source file pair
- Header files should have include guards
- Order of includes:
  1. Related header (for .cpp files)
  2. Project headers
  3. Third-party library headers
  4. Standard library headers

Example:
```cpp
#include "myClass.h"        // Related header

#include "config.h"         // Project headers
#include "utils.h"

#include <spdlog/spdlog.h>  // Third-party headers

#include <vector>           // Standard library
#include <string>
```

## Naming Conventions

### General Rules
- Use descriptive names that clearly indicate purpose
- Avoid abbreviations unless they are well-known (e.g., `id`, `max`, `min`)

### Specific Conventions
- **Classes**: PascalCase (e.g., `CityGraph`, `AStar`, `DubinsInterpolator`)
- **Functions/Methods**: camelCase (e.g., `findPath()`, `getDistance()`, `initializeAgents()`)
- **Variables**: camelCase (e.g., `numCars`, `startPoint`, `selectedIndex`)
- **Constants**: UPPER_SNAKE_CASE (e.g., `CAR_LENGTH`, `ASTAR_MAX_ITERATIONS`)
- **Member variables**: camelCase with no prefix (e.g., `cars`, `numCars`, `graph`)
- **Private members**: No special prefix/suffix needed

### Type Definitions
- Structs with typedef: prefix with underscore, use typedef without underscore
  ```cpp
  typedef struct _cityGraphPoint {
    // ...
  } _cityGraphPoint;
  ```

## Documentation

### File Headers
Every file should start with a Doxygen comment block:
```cpp
/**
 * @file filename.cpp
 * @brief Brief description of the file's purpose
 *
 * More detailed description if needed.
 */
```

### Class Documentation
```cpp
/**
 * @class ClassName
 * @brief Brief description
 *
 * Detailed description of what the class does,
 * its responsibilities, and how it should be used.
 */
class ClassName {
  // ...
};
```

### Function Documentation
```cpp
/**
 * @brief Brief description of what the function does
 *
 * More detailed explanation if needed.
 *
 * @param param1 Description of first parameter
 * @param param2 Description of second parameter
 * @return Description of return value
 */
ReturnType functionName(Type1 param1, Type2 param2);
```

### Inline Comments
- Use inline comments to explain **why**, not **what**
- Comment complex algorithms or non-obvious logic
- Keep comments concise and up-to-date with code changes

## Error Handling

### Logging
- Use spdlog for all logging
- Log levels:
  - `debug`: Detailed information for debugging
  - `info`: General informational messages
  - `warn`: Warning messages for potential issues
  - `error`: Error messages for failures

### Input Validation
- Validate all inputs at function boundaries
- Check array/vector bounds before accessing
- Verify file existence before attempting to open
- Handle filesystem errors gracefully

### Example
```cpp
bool carsCollided(const Car car1, const Car car2, const int time) {
  const std::vector<sf::Vector2f> path1 = car1.getPath();
  const std::vector<sf::Vector2f> path2 = car2.getPath();
  
  // Validate time index is within bounds
  if (time < 0 || time >= static_cast<int>(path1.size()) || 
      time >= static_cast<int>(path2.size())) {
    return false;
  }
  
  // ... rest of function
}
```

## Performance Considerations

### Memory Management
- Use `reserve()` for vectors when size is known in advance
- Prefer references over copies for large objects
- Use move semantics where appropriate

### Algorithmic Efficiency
- Be aware of algorithm complexity (O(n), O(n²), etc.)
- Avoid nested loops where possible
- Cache frequently computed values

### Constants
Key algorithmic constants are defined in `config.h`:
- `ASTAR_MAX_ITERATIONS`: Prevents infinite loops in A*
- `NUM_SPEED_DIVISIONS`: Controls granularity of speed planning
- `COLLISION_SAFETY_FACTOR`: Safety margin for collision detection
- `GRAPH_POINT_DISTANCE`: Spacing of graph nodes

## Code Review Checklist

Before submitting code for review, ensure:
- [ ] Code compiles without warnings
- [ ] Code follows formatting guidelines (run clang-format)
- [ ] All functions have appropriate documentation
- [ ] Magic numbers are replaced with named constants
- [ ] Input validation is present where needed
- [ ] Error cases are handled appropriately
- [ ] Logging is used for important events and errors
- [ ] `const` is used appropriately
- [ ] Variable names are descriptive
- [ ] Code is properly commented where needed

## Additional Resources

- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [SFML Documentation](https://www.sfml-dev.org/documentation/)
- [spdlog Documentation](https://github.com/gabime/spdlog)
- [OMPL Documentation](https://ompl.kavrakilab.org/)
