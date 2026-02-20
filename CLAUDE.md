# Project Guidelines for Claude

## Project Overview

### Phase 1
I would like to add more unit testing to the EbsdLib library. Can we put together a TODO.md file stored in the "Code_Review" directory that lists all the candidate classes to create unit tests for?

### Phase 2
I would also like to write documentation for this library

### Phase 3
I would also like to write more example code that shows how to use the library

How should we get stated with these tasks?

## Directory Structure
- `Source/Ebsdlib/` - Core library 
- `Source/Apps/` - various test applications
- `src/Test/` - Test files
- `cmake/` - CMake configuration


## Directories to Ignore
- `scripts/` - Build/utility scripts
- `conda/` - Conda packaging

## Coding Standards

### C++ Style (from .clang-format)
- C++20 standard
- Allman brace style (braces on new lines for classes, control statements, enums, functions, namespaces, structs, before else)
- 200 column limit
- 2-space indentation, no tabs
- Pointer alignment left (`int* ptr` not `int *ptr`)
- No space before parentheses
- Sort includes alphabetically
- No short functions on single line
- Always break template declarations
- Constructor initializers break before comma

### Naming Conventions (from .clang-tidy)
- C++ header files: `.hpp` extension
- C++ source files: `.cpp` extension
- Namespaces: `lower_case`
- Classes: `CamelCase`
- Structs: `CamelCase`
- Class methods: `camelBack`
- Functions: `camelBack`
- Variables: `camelBack`
- Private members: `m_` prefix + `CamelCase` (e.g., `m_MemberVariable`)
- Global variables: `CamelCase`
- Global constants: `k_` prefix + `CamelCase` (e.g., `k_DefaultValue`)
- Local pointers: `camelBack` + `Ptr` suffix (e.g., `dataPtr`)
- Type aliases: `CamelCase` + `Type` suffix (e.g., `ValueType`)
- Macros: `UPPER_CASE`


## Build System
- vcpkg for dependency management
- CMake-based build system

Example configuring the project
```bash
cd /Users/mjackson/Workspace1/EbsdLib && cmake --preset EbsdLib-Release
```

- Build directory is located at "/Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release"

Example building the project
```bash
cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all
```

- Python anaconda environment 'dream3d' can be used if needed

## Testing
- Unit tests use the Catch2 framework.
- Use the `ctest` to run unit tests

### Running Unit Tests
- Always use `ctest` to run unit tests, NOT the test binary directly
- The `ctest` command handles test data extraction and cleanup automatically
- Use the `-R` flag to run specific tests by name pattern

Example - Running a specific test:
```bash
cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && ctest -R "EbsdLib::FillBadData" --verbose
```

Example - Running all SimplnxCore tests:
```bash
cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && ctest -R "EbsdLib::" --verbose
```

### Printing debug statements in unit tests

## Additional Notes
<!-- Add any other project-specific rules here -->
