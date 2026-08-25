# Repository Guidance

EbsdLib is a C++20 CMake library. Follow the checked-in formatter and lint configuration: Allman braces, two-space indentation, left-aligned pointers, and the repository naming conventions.

- Core library code is under `Source/EbsdLib/`; applications are under `Source/Apps/`; tests and CMake support live in their existing repository directories.
- Configure with `cmake --preset EbsdLib-Release`; verify with the actual CMake build and tests rather than relying on clangd diagnostics alone.
- Run tests through `ctest`, not test executables directly.
- For inverse-pole-figure and standard-stereographic-triangle visualization, reuse the LaueOps stereographic mapping. Do not substitute a Lambert eta/chi rectangle.
- Keep durable architecture and workflow documentation in the repository; do not encode active-branch status or personal machine paths here.
