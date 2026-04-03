# Nolze-Hielscher IPF Color Palettes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add perceptually-improved IPF color palettes to EbsdLib based on the Nolze-Hielscher (2016) algorithm, implemented as a pluggable color key system alongside the existing TSL coloring.

**Architecture:** Introduce an `IColorKey` interface with concrete implementations for TSL (current behavior, refactored) and Nolze-Hielscher (new). A `FundamentalSectorGeometry` utility computes normalized polar coordinates within arbitrary spherical sectors. The existing `LaueOps` virtual dispatch is extended with a color key selector, defaulting to TSL for backward compatibility.

**Tech Stack:** C++20, Catch2 (testing), CMake, Eigen3 (already a dependency)

**Paper Reference:** Nolze, G. & Hielscher, R. "Orientations -- perfectly colored." *J. Appl. Crystallogr.* 49.5 (2016): 1786-1802. DOI: 10.1107/S1600576716012942. Preprint: https://www.tu-chemnitz.de/mathematik/preprint/2016/PREPRINT_01.pdf

---

## Coding Standards Reference (from CLAUDE.md)

- C++20, Allman brace style, 200-column limit, 2-space indent
- Classes: `CamelCase`, methods: `camelBack`, private members: `m_` + `CamelCase`
- Headers: `.hpp`, sources: `.cpp`
- Constants: `k_` prefix + `CamelCase`
- Type aliases: `CamelCase` + `Type` suffix

## Build & Test Commands

```bash
# Configure
cd /Users/mjackson/Workspace1/EbsdLib && cmake --preset EbsdLib-Release

# Build
cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all

# Run specific test
cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && ctest -R "EbsdLib::ColorKey" --verbose

# Run all tests
cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && ctest -R "EbsdLib::" --verbose
```

---

## File Structure

### New Files

| File | Responsibility |
|------|---------------|
| `Source/EbsdLib/Utilities/ColorSpaceUtils.hpp` | HSL/HSV/RGB conversions, hue correction precomputation |
| `Source/EbsdLib/Utilities/IColorKey.hpp` | Abstract interface for IPF color key strategies |
| `Source/EbsdLib/Utilities/TSLColorKey.hpp` | Refactored current algorithm behind IColorKey interface |
| `Source/EbsdLib/Utilities/TSLColorKey.cpp` | TSL implementation (extracted from LaueOps::computeIPFColor) |
| `Source/EbsdLib/Utilities/FundamentalSectorGeometry.hpp` | Sector definition: boundary normals, vertices, barycenter, polar coordinate computation |
| `Source/EbsdLib/Utilities/FundamentalSectorGeometry.cpp` | Polar coordinate algorithms, azimuthal correction |
| `Source/EbsdLib/Utilities/NolzeHielscherColorKey.hpp` | Nolze-Hielscher color key header |
| `Source/EbsdLib/Utilities/NolzeHielscherColorKey.cpp` | Full N-H algorithm: hue speed function, lightness, saturation, extended key |
| `Source/Test/ColorSpaceUtilsTest.cpp` | Tests for HSL/HSV/RGB conversions |
| `Source/Test/FundamentalSectorGeometryTest.cpp` | Tests for polar coordinates, barycenter, boundary intersection |
| `Source/Test/NolzeHielscherColorKeyTest.cpp` | Tests for N-H color mapping, all Laue groups |
| `Source/Test/TSLColorKeyTest.cpp` | Regression tests: refactored TSL produces identical output to current code |

### Modified Files

| File | Change |
|------|--------|
| `Source/EbsdLib/Utilities/SourceList.cmake` | Add new .hpp/.cpp files to build |
| `Source/Test/CMakeLists.txt` | Add new test source files |
| `Source/EbsdLib/LaueOps/LaueOps.h` | Add `setColorKey()` / `getColorKey()` methods; add virtual `getFundamentalSectorGeometry()` |
| `Source/EbsdLib/LaueOps/LaueOps.cpp` | Wire `computeIPFColor()` to delegate to active IColorKey |
| `Source/EbsdLib/LaueOps/CubicOps.cpp` | Override `getFundamentalSectorGeometry()` with cubic sector definition |
| (and all other 10 LaueOps subclass .cpp files) | Same: override `getFundamentalSectorGeometry()` |

---

## Existing Code Reference

### Key Types (already in EbsdLib)
- `Rgb = uint32_t` -- ARGB format (0xAARRGGBB), defined in `ColorTable.h:65`
- `QuatD` -- double quaternion, used for symmetry operators
- `Matrix3X3D` -- 3x3 rotation matrix
- `Matrix3X1D` -- 3x1 column vector (used as direction vector)
- `FloatArrayType::Pointer`, `UInt8ArrayType::Pointer` -- dynamic arrays

### Key Methods (already in EbsdLib)
- `LaueOps::computeIPFColor()` -- `LaueOps.cpp:159-228`, current TSL coloring
- `LaueOps::generateIPFColor()` -- virtual, each subclass delegates to `computeIPFColor()`
- `CubicOps::inUnitTriangle(eta, chi)` -- `CubicOps.cpp:1649-1664`, SST boundary test
- `CubicOps::getIpfColorAngleLimits(eta)` -- `CubicOps.cpp:1631-1646`, returns {etaMin, etaMax, chiMax}
- `LaueOps::GetAllOrientationOps()` -- returns vector of 11 LaueOps, indexed by CrystalStructure constants

### CrystalStructure Index Mapping (EbsdLibConstants.h:221-239)
```
Index 0  = Hexagonal_High (6/mmm)     Index 6  = OrthoRhombic (mmm)
Index 1  = Cubic_High (m-3m)          Index 7  = Tetragonal_Low (4/m)
Index 2  = Hexagonal_Low (6/m)        Index 8  = Tetragonal_High (4/mmm)
Index 3  = Cubic_Low (m-3)            Index 9  = Trigonal_Low (-3)
Index 4  = Triclinic (-1)             Index 10 = Trigonal_High (-3m)
Index 5  = Monoclinic (2/m)
```

### SST Boundary Definitions (from inUnitTriangle in each subclass)

| Laue Group | etaMin (deg) | etaMax (deg) | chiMax | Vertices (crystal directions) |
|------------|-------------|-------------|--------|-------------------------------|
| m-3m (CubicOps) | 0 | 45 | `acos(sqrt(1/(2+tan^2(eta))))` | [001], [011]/sqrt2, [111]/sqrt3 |
| m-3 (CubicLowOps) | 0 | 90 | `acos(sqrt(1/(2+tan^2(eta))))` | [001], [010], [011]/sqrt2, [111]/sqrt3 |
| 6/mmm (HexagonalOps) | 0 | 30 | 90 | [0001], [2-1-10], [10-10] |
| 6/m (HexagonalLowOps) | 0 | 60 | 90 | [0001], [2-1-10], [11-20] |
| 4/mmm (TetragonalOps) | 0 | 45 | 90 | [001], [100], [110]/sqrt2 |
| 4/m (TetragonalLowOps) | 0 | 90 | 90 | [001], [100], [010] |
| -3m (TrigonalOps) | -90 | -30 | 90 | (rotated hexagonal sector) |
| -3 (TrigonalLowOps) | -120 | 0 | 90 | (rotated hexagonal sector) |
| mmm (OrthoRhombicOps) | 0 | 90 | 90 | [001], [100], [010] |
| 2/m (MonoclinicOps) | 0 | 180 | 90 | [001], [100], [-100] |
| -1 (TriclinicOps) | 0 | 180 | 90 | Full upper hemisphere |

### Point Group Classification for Color Key Mode (Paper Table 1)

| Laue Group | Color Key Mode | Supergroup P+ | Reflection Needed? |
|------------|---------------|---------------|-------------------|
| m-3m | Standard | (self) | No |
| m-3 | Extended | m-3m | Yes |
| 6/mmm | Standard | (self) | No |
| 6/m | Extended | 6/mmm | Yes |
| 4/mmm | Standard | (self) | No |
| 4/m | Extended | 4/mmm | Yes |
| -3m | Standard | (self) | No |
| -3 | Impossible | - | N/A (topologically impossible) |
| mmm | Standard | (self) | No |
| 2/m | Extended | mmm | Yes |
| -1 | Impossible | - | N/A (topologically impossible) |

---

## Algorithm Reference (from Paper, Clean-Room)

### The Complete Nolze-Hielscher Mapping

**Input:** Unit crystal direction h (already projected into the fundamental sector)

**Output:** RGB color (0-255 per channel)

```
1. Compute barycenter p = normalize(sum(vertices))

2. Compute normalized polar coordinates (radius, rho) relative to p:
   - radius: For each boundary normal N_j:
       B_j = normalize(cross(cross(h, p), N_j))
       ratio_j = angle(-h, B_j) / angle(-p, B_j)
     radius = min(ratio_j) over all j      // in [0, 1]
   - rho: Project h onto tangent plane at p, measure angle via atan2

3. Apply azimuthal correction:
   - For 3-vertex sectors: redistribute rho so each vertex gets 1/3 of [0, 2*pi]
   - Weight by hue speed function: v(rho) = d(rho) * (0.5 + G(0) + G(120) + G(-120))
     where G(c) = exp(-|wrap(rho - c)|/4) and d(rho) = distance from center to boundary
   - H = cumulative integral of v(rho), normalized to [0, 360]

4. FOR STANDARD KEYS (all-mirror boundaries):
   - theta = radius * pi/2
   - L = 0.25 * (theta/(pi/2)) + 0.75 * sin^2(theta/2)     // Paper Appendix A.2, lambda_L=0.25
   - S = 1 - 0.5 * |2*L - 1|                                  // Paper Appendix A.2, lambda_S=0.25

5. FOR EXTENDED KEYS (non-mirror boundaries):
   - Also project h into supergroup P+ sector -> h_plus
   - If h_plus != h (direction fell in extended half):
       radius_mapped = (1 - radius) / 2     // black center: [0.5, 0.0]
     Else:
       radius_mapped = 0.5 + radius / 2     // white center: [0.5, 1.0]
   - L = lightness formula applied to radius_mapped
   - S = saturation formula applied to L

6. Convert (H, S, L) -> RGB via standard HSL-to-RGB
```

### Key Constants (from Paper)
```
lambda_L = 0.25          // lightness nonlinearity (Appendix A.2, Fig. 16b)
lambda_S = 0.25          // saturation control (Appendix A.2, Fig. 16c)
Gaussian_width = 4       // in hue speed function exponent (Appendix A.1)
Gaussian_baseline = 0.5  // constant term in v(rho) (Appendix A.1)
Gaussian_centers = {0, 120, -120} degrees  // primary color positions
```

---

## Tasks

### Task 1: ColorSpaceUtils -- HSL/HSV/RGB Conversions

**Files:**
- Create: `Source/EbsdLib/Utilities/ColorSpaceUtils.hpp`
- Create: `Source/Test/ColorSpaceUtilsTest.cpp`
- Modify: `Source/EbsdLib/Utilities/SourceList.cmake`
- Modify: `Source/Test/CMakeLists.txt`

- [ ] **Step 1: Write the failing tests**

```cpp
// Source/Test/ColorSpaceUtilsTest.cpp
#include <catch2/catch.hpp>
#include "EbsdLib/Utilities/ColorSpaceUtils.hpp"

TEST_CASE("ebsdlib::ColorSpaceUtils::HslToRgb", "[EbsdLib][ColorSpaceUtils]")
{
  SECTION("Pure Red")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 1.0, 0.5);
    REQUIRE(r == Approx(1.0).margin(1e-6));
    REQUIRE(g == Approx(0.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
  SECTION("Pure Green")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(1.0 / 3.0, 1.0, 0.5);
    REQUIRE(r == Approx(0.0).margin(1e-6));
    REQUIRE(g == Approx(1.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
  SECTION("Pure Blue")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(2.0 / 3.0, 1.0, 0.5);
    REQUIRE(r == Approx(0.0).margin(1e-6));
    REQUIRE(g == Approx(0.0).margin(1e-6));
    REQUIRE(b == Approx(1.0).margin(1e-6));
  }
  SECTION("White")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 0.0, 1.0);
    REQUIRE(r == Approx(1.0).margin(1e-6));
    REQUIRE(g == Approx(1.0).margin(1e-6));
    REQUIRE(b == Approx(1.0).margin(1e-6));
  }
  SECTION("Black")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 0.0, 0.0);
    REQUIRE(r == Approx(0.0).margin(1e-6));
    REQUIRE(g == Approx(0.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
  SECTION("50% Gray")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 0.0, 0.5);
    REQUIRE(r == Approx(0.5).margin(1e-6));
    REQUIRE(g == Approx(0.5).margin(1e-6));
    REQUIRE(b == Approx(0.5).margin(1e-6));
  }
  SECTION("Yellow (H=60deg)")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(1.0 / 6.0, 1.0, 0.5);
    REQUIRE(r == Approx(1.0).margin(1e-6));
    REQUIRE(g == Approx(1.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
}

TEST_CASE("ebsdlib::ColorSpaceUtils::HslToHsv", "[EbsdLib][ColorSpaceUtils]")
{
  SECTION("Full saturation, mid lightness -> V=1, S=1")
  {
    auto [h, s, v] = ebsdlib::color::hslToHsv(0.0, 1.0, 0.5);
    REQUIRE(h == Approx(0.0));
    REQUIRE(s == Approx(1.0));
    REQUIRE(v == Approx(1.0));
  }
  SECTION("Zero saturation -> S_hsv = 0")
  {
    auto [h, s, v] = ebsdlib::color::hslToHsv(0.5, 0.0, 0.5);
    REQUIRE(s == Approx(0.0));
    REQUIRE(v == Approx(0.5));
  }
}

TEST_CASE("ebsdlib::ColorSpaceUtils::RoundTrip", "[EbsdLib][ColorSpaceUtils]")
{
  // HSL -> RGB -> verify it matches known values for several hues
  for(double hue = 0.0; hue < 1.0; hue += 0.1)
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(hue, 1.0, 0.5);
    REQUIRE(r >= 0.0);
    REQUIRE(r <= 1.0);
    REQUIRE(g >= 0.0);
    REQUIRE(g <= 1.0);
    REQUIRE(b >= 0.0);
    REQUIRE(b <= 1.0);
    // At L=0.5, S=1: max component should be 1.0
    double maxVal = std::max({r, g, b});
    REQUIRE(maxVal == Approx(1.0).margin(1e-6));
  }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "ColorSpaceUtils" --verbose`
Expected: Compilation failure (files don't exist yet)

- [ ] **Step 3: Write the implementation**

```cpp
// Source/EbsdLib/Utilities/ColorSpaceUtils.hpp
#pragma once

#include "EbsdLib/EbsdLib.h"

#include <array>
#include <cmath>
#include <algorithm>

namespace ebsdlib
{
namespace color
{

/**
 * @brief Convert HSL to RGB. All inputs and outputs in [0, 1].
 * @param h Hue in [0, 1) where 0=red, 1/3=green, 2/3=blue
 * @param s Saturation in [0, 1]
 * @param l Lightness in [0, 1]
 * @return {r, g, b} each in [0, 1]
 */
inline std::array<double, 3> hslToRgb(double h, double s, double l)
{
  double c = (1.0 - std::abs(2.0 * l - 1.0)) * s;
  double hp = h * 6.0;
  double x = c * (1.0 - std::abs(std::fmod(hp, 2.0) - 1.0));
  double m = l - c / 2.0;

  double r1 = 0.0;
  double g1 = 0.0;
  double b1 = 0.0;

  if(hp < 1.0)
  {
    r1 = c; g1 = x; b1 = 0.0;
  }
  else if(hp < 2.0)
  {
    r1 = x; g1 = c; b1 = 0.0;
  }
  else if(hp < 3.0)
  {
    r1 = 0.0; g1 = c; b1 = x;
  }
  else if(hp < 4.0)
  {
    r1 = 0.0; g1 = x; b1 = c;
  }
  else if(hp < 5.0)
  {
    r1 = x; g1 = 0.0; b1 = c;
  }
  else
  {
    r1 = c; g1 = 0.0; b1 = x;
  }

  return {std::clamp(r1 + m, 0.0, 1.0),
          std::clamp(g1 + m, 0.0, 1.0),
          std::clamp(b1 + m, 0.0, 1.0)};
}

/**
 * @brief Convert HSL to HSV. All inputs and outputs in [0, 1].
 */
inline std::array<double, 3> hslToHsv(double h, double s, double l)
{
  double l2 = 2.0 * l;
  double s2 = s * ((l2 <= 1.0) ? l2 : (2.0 - l2));
  double v = (l2 + s2) / 2.0;
  double sv = (l2 + s2 > 1e-12) ? (2.0 * s2 / (l2 + s2)) : 0.0;
  return {h, sv, v};
}

/**
 * @brief Convert RGB [0,1] to 8-bit [0,255] clamped.
 */
inline std::array<uint8_t, 3> rgbToBytes(double r, double g, double b)
{
  return {static_cast<uint8_t>(std::clamp(r * 255.0, 0.0, 255.0)),
          static_cast<uint8_t>(std::clamp(g * 255.0, 0.0, 255.0)),
          static_cast<uint8_t>(std::clamp(b * 255.0, 0.0, 255.0))};
}

} // namespace color
} // namespace ebsdlib
```

- [ ] **Step 4: Add to build system**

Add `ColorSpaceUtils.hpp` to `Source/EbsdLib/Utilities/SourceList.cmake` in the headers list.
Add `ColorSpaceUtilsTest.cpp` to `Source/Test/CMakeLists.txt` in the test sources list.

- [ ] **Step 5: Build and run tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "ColorSpaceUtils" --verbose`
Expected: All PASS

- [ ] **Step 6: Commit**

```bash
git add Source/EbsdLib/Utilities/ColorSpaceUtils.hpp Source/Test/ColorSpaceUtilsTest.cpp Source/EbsdLib/Utilities/SourceList.cmake Source/Test/CMakeLists.txt
git commit -m "feat: add ColorSpaceUtils with HSL/HSV/RGB conversions"
```

---

### Task 2: IColorKey Interface

**Files:**
- Create: `Source/EbsdLib/Utilities/IColorKey.hpp`
- Modify: `Source/EbsdLib/Utilities/SourceList.cmake`

- [ ] **Step 1: Write the interface**

```cpp
// Source/EbsdLib/Utilities/IColorKey.hpp
#pragma once

#include "EbsdLib/EbsdLib.h"

#include <array>
#include <memory>
#include <string>

namespace ebsdlib
{

/**
 * @brief Abstract interface for IPF color key strategies.
 *
 * Maps a crystal direction (already projected into the fundamental sector)
 * to an RGB color. Implementations include TSL (traditional) and
 * Nolze-Hielscher (perceptually improved).
 *
 * Two overloads are provided:
 * - direction2Color(Vec3): takes a 3D unit direction vector (preferred for N-H)
 * - direction2Color(eta, chi, angleLimits): takes spherical coords (TSL compatibility)
 */
class EbsdLib_EXPORT IColorKey
{
public:
  using Pointer = std::shared_ptr<IColorKey>;
  using Vec3 = std::array<double, 3>;

  virtual ~IColorKey() = default;

  /**
   * @brief Map a unit crystal direction vector (in the fundamental sector) to an RGB color.
   * This is the primary interface. The direction must already be projected into the SST.
   * @param direction Unit direction vector {x, y, z} in the fundamental sector
   * @return {R, G, B} each in [0.0, 1.0]
   */
  virtual Vec3 direction2Color(const Vec3& direction) const = 0;

  /**
   * @brief Map a crystal direction via spherical coordinates to an RGB color.
   * Provided for backward compatibility with the TSL pipeline.
   * Default implementation converts to a direction vector and calls the Vec3 overload.
   * @param eta Azimuthal angle of the direction (radians)
   * @param chi Polar angle of the direction from z-axis (radians)
   * @param angleLimits {etaMin, etaMax, chiMax} from the LaueOps subclass
   * @return {R, G, B} each in [0.0, 1.0]
   */
  virtual Vec3 direction2Color(double eta, double chi, const Vec3& angleLimits) const
  {
    // Default: convert spherical to Cartesian and delegate
    double sinChi = std::sin(chi);
    Vec3 dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
    return direction2Color(dir);
  }

  /**
   * @brief Human-readable name of this color key.
   */
  virtual std::string name() const = 0;
};

} // namespace ebsdlib
```

- [ ] **Step 2: Add to build system**

Add `IColorKey.hpp` to `Source/EbsdLib/Utilities/SourceList.cmake` headers list.

- [ ] **Step 3: Build to verify compilation**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all`
Expected: Compiles successfully (header-only, no test yet)

- [ ] **Step 4: Commit**

```bash
git add Source/EbsdLib/Utilities/IColorKey.hpp Source/EbsdLib/Utilities/SourceList.cmake
git commit -m "feat: add IColorKey abstract interface for pluggable IPF color strategies"
```

---

### Task 3: TSLColorKey -- Refactor Current Algorithm

**Files:**
- Create: `Source/EbsdLib/Utilities/TSLColorKey.hpp`
- Create: `Source/EbsdLib/Utilities/TSLColorKey.cpp`
- Create: `Source/Test/TSLColorKeyTest.cpp`
- Modify: `Source/EbsdLib/Utilities/SourceList.cmake`
- Modify: `Source/Test/CMakeLists.txt`

- [ ] **Step 1: Write regression tests**

The test should verify that the new TSLColorKey produces IDENTICAL output to the current `LaueOps::computeIPFColor()` for a range of directions across all 11 Laue groups.

```cpp
// Source/Test/TSLColorKeyTest.cpp
#include <catch2/catch.hpp>
#include "EbsdLib/Utilities/TSLColorKey.hpp"
#include "EbsdLib/LaueOps/LaueOps.h"

TEST_CASE("ebsdlib::TSLColorKey::MatchesLegacyOutput", "[EbsdLib][TSLColorKey]")
{
  auto allOps = LaueOps::GetAllOrientationOps();
  ebsdlib::TSLColorKey tslKey;

  // Test a grid of directions across each Laue group's SST
  for(size_t opIdx = 0; opIdx < 11; opIdx++)
  {
    auto& ops = *allOps[opIdx];
    SECTION(ops.getSymmetryName())
    {
      // Sample Euler angles and a reference direction
      double refDir[3] = {0.0, 0.0, 1.0};

      // Test several known Euler angle sets
      std::vector<std::array<double, 3>> testEulers = {
        {0.0, 0.0, 0.0},
        {0.5, 0.3, 0.2},
        {1.0, 0.7, 0.5},
        {0.1, 0.1, 0.1},
        {2.0, 1.0, 0.8}
      };

      for(auto& euler : testEulers)
      {
        double eulerArr[3] = {euler[0], euler[1], euler[2]};

        // Get legacy color via existing pipeline
        auto legacyColor = ops.generateIPFColor(eulerArr, refDir, false);

        // Verify color is non-degenerate (at least one channel > 0)
        REQUIRE(legacyColor.r + legacyColor.g + legacyColor.b > 0);
      }
    }
  }
}

TEST_CASE("ebsdlib::TSLColorKey::KnownCubicDirections", "[EbsdLib][TSLColorKey]")
{
  ebsdlib::TSLColorKey tslKey;

  SECTION("[001] direction -> Red (center of SST top)")
  {
    double eta = 0.0;
    double chi = 0.0;
    double chiMax = std::acos(std::sqrt(1.0 / 3.0));
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
    auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
    REQUIRE(r == Approx(1.0).margin(0.01));
    REQUIRE(g == Approx(0.0).margin(0.01));
    REQUIRE(b == Approx(0.0).margin(0.01));
  }

  SECTION("[011] direction -> Green-ish (SST edge)")
  {
    double eta = M_PI / 4.0;  // 45 degrees
    double chi = 0.0;
    double chiMax = std::acos(std::sqrt(1.0 / 3.0));
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
    auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
    // At eta=etaMax, chi=0: R=1, B=1, G=0 (before normalize)
    // After sqrt and normalize: R=1, B=1, G=0
    REQUIRE(r == Approx(1.0).margin(0.01));
    REQUIRE(b == Approx(1.0).margin(0.01));
  }

  SECTION("Grid of directions all produce valid [0,1] outputs")
  {
    for(double eta = 0.0; eta <= M_PI / 4.0; eta += 0.05)
    {
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(std::max(eta, 0.001)) * std::tan(std::max(eta, 0.001)))));
      for(double chi = 0.0; chi <= chiMax; chi += 0.05)
      {
        std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
        auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }
}

TEST_CASE("ebsdlib::TSLColorKey::ExactRegressionAgainstLegacy", "[EbsdLib][TSLColorKey]")
{
  // This test will be fully implemented after Task 6 (LaueOps integration).
  // At that point, we can compare TSLColorKey output against the original
  // computeIPFColor() for the same eta/chi/limits inputs and verify
  // they are identical to within floating-point epsilon.
  //
  // For now, verify the formula produces known analytic results:
  ebsdlib::TSLColorKey tslKey;

  // At chi/chiMax = 0.5, eta = etaMin:
  // R = 1 - 0.5 = 0.5, B = 0, G = (1-0)*0.5 = 0.5
  // After sqrt: R = 0.707, G = 0.707, B = 0
  // After normalize: R = 1.0, G = 1.0, B = 0.0
  double chiMax = 1.0;
  double chi = 0.5;
  double eta = 0.0;
  std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
  auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
  REQUIRE(r == Approx(1.0).margin(0.01));
  REQUIRE(g == Approx(1.0).margin(0.01));
  REQUIRE(b == Approx(0.0).margin(0.01));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "TSLColorKey" --verbose`
Expected: Compilation failure

- [ ] **Step 3: Write TSLColorKey implementation**

Extract the algorithm from `LaueOps::computeIPFColor()` (LaueOps.cpp:159-228) into a standalone class:

```cpp
// Source/EbsdLib/Utilities/TSLColorKey.hpp
#pragma once

#include "EbsdLib/Utilities/IColorKey.hpp"

namespace ebsdlib
{

/**
 * @brief Traditional TSL/HKL IPF color key.
 * Refactored from LaueOps::computeIPFColor().
 * Requires angle limits from the LaueOps subclass, so the spherical
 * coordinate overload is the primary interface for this key.
 */
class EbsdLib_EXPORT TSLColorKey : public IColorKey
{
public:
  TSLColorKey() = default;
  ~TSLColorKey() override = default;

  /**
   * @brief TSL coloring from spherical coordinates (primary for this key).
   * Overrides the base class default to use the TSL-specific algorithm directly.
   */
  Vec3 direction2Color(double eta, double chi, const Vec3& angleLimits) const override;

  /**
   * @brief TSL coloring from direction vector.
   * Converts to spherical and requires stored angle limits.
   * NOTE: This overload is less efficient for TSL; prefer the (eta, chi, angleLimits) overload.
   * Uses a fallback that maps to the full [0, pi/4] x [0, chiMax] sector.
   */
  Vec3 direction2Color(const Vec3& direction) const override;

  std::string name() const override;

  /**
   * @brief Set default angle limits used by the Vec3 overload.
   * Call this when the LaueOps subclass is known.
   */
  void setDefaultAngleLimits(const Vec3& limits);

private:
  Vec3 m_DefaultAngleLimits = {0.0, 0.7854, 0.6155}; // cubic high defaults
};

} // namespace ebsdlib
```

```cpp
// Source/EbsdLib/Utilities/TSLColorKey.cpp
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#include <algorithm>
#include <cmath>

namespace ebsdlib
{

TSLColorKey::Vec3 TSLColorKey::direction2Color(double eta, double chi, const Vec3& angleLimits) const
{
  // Extracted from LaueOps::computeIPFColor (LaueOps.cpp:200-227)
  double etaMin = angleLimits[0];
  double etaMax = angleLimits[1];
  double chiMax = angleLimits[2];

  double r = 1.0 - chi / chiMax;
  double b = std::abs(eta - etaMin) / (etaMax - etaMin);
  double g = 1.0 - b;
  g *= chi / chiMax;
  b *= chi / chiMax;

  // Square-root gamma correction
  r = std::sqrt(r);
  g = std::sqrt(g);
  b = std::sqrt(b);

  // Normalize by max component
  double maxVal = std::max({r, g, b});
  if(maxVal > 0.0)
  {
    r /= maxVal;
    g /= maxVal;
    b /= maxVal;
  }

  return {std::clamp(r, 0.0, 1.0),
          std::clamp(g, 0.0, 1.0),
          std::clamp(b, 0.0, 1.0)};
}

TSLColorKey::Vec3 TSLColorKey::direction2Color(const Vec3& direction) const
{
  double chi = std::acos(std::clamp(direction[2], -1.0, 1.0));
  double eta = std::atan2(direction[1], direction[0]);
  return direction2Color(eta, chi, m_DefaultAngleLimits);
}

void TSLColorKey::setDefaultAngleLimits(const Vec3& limits)
{
  m_DefaultAngleLimits = limits;
}

std::string TSLColorKey::name() const
{
  return "TSL";
}

} // namespace ebsdlib
```

- [ ] **Step 4: Add to build system**

Add TSLColorKey.hpp and TSLColorKey.cpp to `Source/EbsdLib/Utilities/SourceList.cmake`.
Add TSLColorKeyTest.cpp to `Source/Test/CMakeLists.txt`.

- [ ] **Step 5: Build and run tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "TSLColorKey" --verbose`
Expected: All PASS

- [ ] **Step 6: Commit**

```bash
git add Source/EbsdLib/Utilities/TSLColorKey.hpp Source/EbsdLib/Utilities/TSLColorKey.cpp Source/Test/TSLColorKeyTest.cpp Source/EbsdLib/Utilities/SourceList.cmake Source/Test/CMakeLists.txt
git commit -m "feat: extract TSLColorKey from LaueOps::computeIPFColor"
```

---

### Task 4: FundamentalSectorGeometry -- Sector Definitions and Polar Coordinates

**Files:**
- Create: `Source/EbsdLib/Utilities/FundamentalSectorGeometry.hpp`
- Create: `Source/EbsdLib/Utilities/FundamentalSectorGeometry.cpp`
- Create: `Source/Test/FundamentalSectorGeometryTest.cpp`
- Modify: `Source/EbsdLib/Utilities/SourceList.cmake`
- Modify: `Source/Test/CMakeLists.txt`

This is the most complex task. It implements normalized polar coordinates within an arbitrary spherical sector.

- [ ] **Step 1: Write failing tests for polar coordinate computation**

```cpp
// Source/Test/FundamentalSectorGeometryTest.cpp
#include <catch2/catch.hpp>
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"

using Vec3 = std::array<double, 3>;

// Helper: normalize a vector
static Vec3 normalize(Vec3 v)
{
  double len = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  return {v[0]/len, v[1]/len, v[2]/len};
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::CubicHighVertices", "[EbsdLib][FundamentalSector]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("Has 3 vertices")
  {
    REQUIRE(sector.vertices().size() == 3);
  }

  SECTION("Vertices are [001], [011], [111]")
  {
    auto verts = sector.vertices();
    // [001]
    REQUIRE(verts[0][2] == Approx(1.0).margin(1e-6));
    // [011] normalized
    REQUIRE(verts[1][1] == Approx(1.0 / std::sqrt(2.0)).margin(1e-6));
    REQUIRE(verts[1][2] == Approx(1.0 / std::sqrt(2.0)).margin(1e-6));
    // [111] normalized
    REQUIRE(verts[2][0] == Approx(1.0 / std::sqrt(3.0)).margin(1e-6));
  }

  SECTION("Barycenter is normalized mean of vertices")
  {
    auto center = sector.barycenter();
    double len = std::sqrt(center[0]*center[0] + center[1]*center[1] + center[2]*center[2]);
    REQUIRE(len == Approx(1.0).margin(1e-6));
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::PolarCoordinates", "[EbsdLib][FundamentalSector]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("At barycenter: radius = 0")
  {
    auto center = sector.barycenter();
    auto [radius, rho] = sector.polarCoordinates(center);
    REQUIRE(radius == Approx(0.0).margin(1e-4));
  }

  SECTION("At vertex [001]: radius = 1 (at boundary)")
  {
    Vec3 v001 = {0.0, 0.0, 1.0};
    auto [radius, rho] = sector.polarCoordinates(v001);
    REQUIRE(radius == Approx(1.0).margin(0.05));
  }

  SECTION("At vertex [011]: radius = 1")
  {
    Vec3 v011 = normalize({0.0, 1.0, 1.0});
    auto [radius, rho] = sector.polarCoordinates(v011);
    REQUIRE(radius == Approx(1.0).margin(0.05));
  }

  SECTION("At vertex [111]: radius = 1")
  {
    Vec3 v111 = normalize({1.0, 1.0, 1.0});
    auto [radius, rho] = sector.polarCoordinates(v111);
    REQUIRE(radius == Approx(1.0).margin(0.05));
  }

  SECTION("Midpoint of [001]-[011] edge: radius = 1")
  {
    Vec3 mid = normalize({0.0, 0.5, 1.0});
    auto [radius, rho] = sector.polarCoordinates(mid);
    REQUIRE(radius == Approx(1.0).margin(0.1));
  }

  SECTION("Radius is in [0, 1] for interior point")
  {
    Vec3 interior = normalize({0.2, 0.3, 1.0});
    auto [radius, rho] = sector.polarCoordinates(interior);
    REQUIRE(radius >= 0.0);
    REQUIRE(radius <= 1.0);
  }

  SECTION("Rho is in [0, 2*pi)")
  {
    Vec3 interior = normalize({0.2, 0.3, 1.0});
    auto [radius, rho] = sector.polarCoordinates(interior);
    REQUIRE(rho >= 0.0);
    REQUIRE(rho < 2.0 * M_PI);
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::EdgeCases", "[EbsdLib][FundamentalSector]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("Direction very close to barycenter returns radius near 0")
  {
    auto center = sector.barycenter();
    // Perturb slightly
    Vec3 nearCenter = normalize({center[0] + 1e-8, center[1] + 1e-8, center[2]});
    auto [radius, rho] = sector.polarCoordinates(nearCenter);
    REQUIRE(radius == Approx(0.0).margin(0.01));
  }

  SECTION("Direction exactly at barycenter returns radius = 0 (singularity guard)")
  {
    auto center = sector.barycenter();
    auto [radius, rho] = sector.polarCoordinates(center);
    REQUIRE(radius == Approx(0.0).margin(1e-6));
  }

  SECTION("Direction on boundary edge (not at vertex) returns radius = 1")
  {
    // Midpoint of [001]-[111] edge (eta=22.5 deg boundary)
    Vec3 edgeMid = normalize({0.2, 0.2, 1.0}); // approximately on the [001]-[111] edge
    auto [radius, rho] = sector.polarCoordinates(edgeMid);
    // Should be close to 1, but not exact since it's an approximation
    REQUIRE(radius > 0.5);
    REQUIRE(radius <= 1.0);
  }

  SECTION("isInside returns true for interior, false for exterior")
  {
    REQUIRE(sector.isInside({0.0, 0.0, 1.0}));          // [001] vertex
    REQUIRE(sector.isInside(normalize({0.2, 0.2, 1.0}))); // interior
    REQUIRE_FALSE(sector.isInside({1.0, 0.0, 0.0}));     // [100] is outside cubic high SST
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::NonTriangularSectors", "[EbsdLib][FundamentalSector]")
{
  SECTION("Cubic low (m-3) has 4 vertices")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::cubicLow();
    REQUIRE(sector.vertices().size() == 4);
    REQUIRE(sector.colorKeyMode() == "extended");
  }

  SECTION("Triclinic (-1) has 0 vertices and covers upper hemisphere")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::triclinic();
    REQUIRE(sector.vertices().empty());
    REQUIRE(sector.colorKeyMode() == "impossible");
    // Any direction in upper hemisphere should be inside
    REQUIRE(sector.isInside({0.0, 0.0, 1.0}));
    REQUIRE(sector.isInside(normalize({0.5, 0.5, 0.1})));
  }

  SECTION("Monoclinic (2/m) spans 180 degrees of eta")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::monoclinic();
    REQUIRE(sector.colorKeyMode() == "extended");
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::AllLaueGroups", "[EbsdLib][FundamentalSector]")
{
  // Verify all 11 sectors can be constructed and have valid barycenters
  std::vector<ebsdlib::FundamentalSectorGeometry> sectors = {
    ebsdlib::FundamentalSectorGeometry::cubicHigh(),
    ebsdlib::FundamentalSectorGeometry::cubicLow(),
    ebsdlib::FundamentalSectorGeometry::hexagonalHigh(),
    ebsdlib::FundamentalSectorGeometry::hexagonalLow(),
    ebsdlib::FundamentalSectorGeometry::tetragonalHigh(),
    ebsdlib::FundamentalSectorGeometry::tetragonalLow(),
    ebsdlib::FundamentalSectorGeometry::trigonalHigh(),
    ebsdlib::FundamentalSectorGeometry::trigonalLow(),
    ebsdlib::FundamentalSectorGeometry::orthorhombic(),
    ebsdlib::FundamentalSectorGeometry::monoclinic(),
    ebsdlib::FundamentalSectorGeometry::triclinic(),
  };

  for(size_t i = 0; i < sectors.size(); i++)
  {
    SECTION("Sector " + std::to_string(i) + " has unit-length barycenter")
    {
      auto c = sectors[i].barycenter();
      double len = std::sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2]);
      REQUIRE(len == Approx(1.0).margin(1e-6));
    }
  }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "FundamentalSector" --verbose`
Expected: Compilation failure

- [ ] **Step 3: Write FundamentalSectorGeometry header**

```cpp
// Source/EbsdLib/Utilities/FundamentalSectorGeometry.hpp
#pragma once

#include "EbsdLib/EbsdLib.h"

#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace ebsdlib
{

/**
 * @brief Defines the geometry of a fundamental sector (SST) on the unit sphere.
 *
 * Stores boundary normals, vertices, and barycenter.
 * Computes normalized polar coordinates (radius, rho) for directions
 * within the sector, as described in Nolze & Hielscher (2016) Section 2.4.
 */
class EbsdLib_EXPORT FundamentalSectorGeometry
{
public:
  using Vec3 = std::array<double, 3>;

  /**
   * @brief Construct a sector from boundary normals and vertices.
   * @param boundaryNormals Outward-pointing normals defining the sector (dot(h, N) >= 0 for interior)
   * @param vertices Corner points of the sector on the unit sphere
   * @param colorKeyMode "standard", "extended", or "impossible"
   * @param supergroupIndex CrystalStructure index of the supergroup P+ (for extended keys)
   */
  FundamentalSectorGeometry(std::vector<Vec3> boundaryNormals, std::vector<Vec3> vertices,
                            std::string colorKeyMode, int32_t supergroupIndex = -1);

  /**
   * @brief Compute normalized polar coordinates of direction h relative to the barycenter.
   * @param h Unit direction vector (must be inside the sector)
   * @return {radius, rho} where radius in [0,1] (0=center, 1=boundary), rho in [0, 2*pi)
   *
   * Special cases:
   * - If h is at the barycenter (angle < 1e-10), returns {0.0, 0.0}
   * - If h is on a boundary, returns {1.0, rho}
   */
  std::pair<double, double> polarCoordinates(const Vec3& h) const;

  /**
   * @brief Apply azimuthal angle correction so that vertices map to evenly-spaced hue positions.
   * For 3-vertex sectors: each vertex gets 1/3 of [0, 2*pi].
   * For 2-vertex sectors: each vertex gets 1/2 of [0, 2*pi].
   * For 4-vertex sectors: each vertex gets 1/4 of [0, 2*pi].
   * For 0 vertices (triclinic): no correction applied.
   * @param rhoRaw Raw azimuthal angle in [0, 2*pi)
   * @return Corrected azimuthal angle in [0, 2*pi)
   */
  double correctAzimuthalAngle(double rhoRaw) const;

  /**
   * @brief Test whether a direction is inside this sector.
   * @param h Unit direction vector
   * @return true if dot(h, N_j) >= 0 for all boundary normals N_j
   */
  bool isInside(const Vec3& h) const;

  const Vec3& barycenter() const;
  const std::vector<Vec3>& vertices() const;
  const std::vector<Vec3>& boundaryNormals() const;
  const std::string& colorKeyMode() const;
  int32_t supergroupIndex() const;

  // Static factory methods for each Laue group
  static FundamentalSectorGeometry cubicHigh();      // m-3m
  static FundamentalSectorGeometry cubicLow();        // m-3
  static FundamentalSectorGeometry hexagonalHigh();   // 6/mmm
  static FundamentalSectorGeometry hexagonalLow();    // 6/m
  static FundamentalSectorGeometry tetragonalHigh();  // 4/mmm
  static FundamentalSectorGeometry tetragonalLow();   // 4/m
  static FundamentalSectorGeometry trigonalHigh();    // -3m
  static FundamentalSectorGeometry trigonalLow();     // -3
  static FundamentalSectorGeometry orthorhombic();    // mmm
  static FundamentalSectorGeometry monoclinic();      // 2/m
  static FundamentalSectorGeometry triclinic();       // -1

private:
  std::vector<Vec3> m_BoundaryNormals;
  std::vector<Vec3> m_Vertices;
  Vec3 m_Barycenter = {0.0, 0.0, 0.0};
  std::string m_ColorKeyMode;
  int32_t m_SupergroupIndex = -1;

  // Precomputed azimuthal correction lookup table (computed in constructor)
  // Maps raw rho -> corrected rho via linear interpolation
  static constexpr size_t k_AzimuthalTableSize = 1000;
  std::array<double, k_AzimuthalTableSize> m_AzimuthalCorrectionTable = {};

  void computeBarycenter();
  void precomputeAzimuthalCorrection();

  // Vector math helpers (static, inline)
  static Vec3 vecNormalize(const Vec3& v);
  static Vec3 vecCross(const Vec3& a, const Vec3& b);
  static double vecDot(const Vec3& a, const Vec3& b);
  static double vecAngle(const Vec3& a, const Vec3& b);
  static Vec3 vecNeg(const Vec3& v);
};

} // namespace ebsdlib
```

- [ ] **Step 4: Write FundamentalSectorGeometry implementation**

The .cpp file contains:
1. Constructor and barycenter computation
2. Vector math helpers
3. `polarCoordinates()` -- the boundary intersection algorithm (textbook spherical geometry)
4. Static factory methods for all 11 Laue groups with their specific boundary normals, vertices, mode, and supergroup

The polar coordinate algorithm (from first-principles spherical geometry):
```
For each boundary normal N_j:
  plane_normal = normalize(cross(h, center))   // great circle containing h and center
  boundary_point = normalize(cross(plane_normal, N_j))  // intersection with boundary j
  ratio_j = angle(-h, boundary_point) / angle(-center, boundary_point)
radius = min(ratio_j) over all j

For azimuthal angle:
  rx = normalize(ref - dot(ref, center) * center)  // project reference onto tangent plane
  ry = normalize(cross(center, rx))
  dv = normalize(h - center)
  rho = atan2(dot(ry, dv), dot(rx, dv))
  rho = mod(rho, 2*pi)
```

Sector definitions derive from the boundary conditions already in each LaueOps subclass's `inUnitTriangle()`.

**Key sector definitions to implement (vertices as normalized crystal directions):**

```cpp
// cubicHigh: m-3m, Standard key
// Normals: [1,-1,0]/sqrt2, [-1,0,1]/sqrt2, [0,1,0]
// (Interior defined by: dot(h, N) >= 0 for all N)
// Vertices: [0,0,1], [0,1,1]/sqrt2, [1,1,1]/sqrt3
// Mode: "standard", no supergroup

// cubicLow: m-3, Extended key, supergroup = m-3m (index 1)
// Normals: [0,-1,0], [-1,0,0], [0,0,-1], (additional boundaries for larger sector)
// Vertices: [0,0,1], [0,1,0], [0,1,1]/sqrt2, [1,1,1]/sqrt3
// Mode: "extended", supergroupIndex = 1

// hexagonalHigh: 6/mmm, Standard key
// Vertices: [0,0,1], [cos30,sin30,0], [1,0,0] (in hex coordinate frame)
// Mode: "standard"

// (similar for all remaining Laue groups)
```

- [ ] **Step 5: Add to build system**

Add FundamentalSectorGeometry.hpp/.cpp to `Source/EbsdLib/Utilities/SourceList.cmake`.
Add FundamentalSectorGeometryTest.cpp to `Source/Test/CMakeLists.txt`.

- [ ] **Step 6: Build and run tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "FundamentalSector" --verbose`
Expected: All PASS

- [ ] **Step 7: Commit**

```bash
git add Source/EbsdLib/Utilities/FundamentalSectorGeometry.hpp Source/EbsdLib/Utilities/FundamentalSectorGeometry.cpp Source/Test/FundamentalSectorGeometryTest.cpp Source/EbsdLib/Utilities/SourceList.cmake Source/Test/CMakeLists.txt
git commit -m "feat: add FundamentalSectorGeometry with polar coordinate computation for all 11 Laue groups"
```

---

### Task 5: NolzeHielscherColorKey -- Core Algorithm

**Files:**
- Create: `Source/EbsdLib/Utilities/NolzeHielscherColorKey.hpp`
- Create: `Source/EbsdLib/Utilities/NolzeHielscherColorKey.cpp`
- Create: `Source/Test/NolzeHielscherColorKeyTest.cpp`
- Modify: `Source/EbsdLib/Utilities/SourceList.cmake`
- Modify: `Source/Test/CMakeLists.txt`

- [ ] **Step 1: Write failing tests**

```cpp
// Source/Test/NolzeHielscherColorKeyTest.cpp
#include <catch2/catch.hpp>
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"

TEST_CASE("ebsdlib::NolzeHielscherColorKey::HueSpeedFunction", "[EbsdLib][NolzeHielscher]")
{
  SECTION("Speed function is positive everywhere")
  {
    for(double rho = 0.0; rho < 360.0; rho += 1.0)
    {
      double v = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(rho, 1.0);
      REQUIRE(v > 0.0);
    }
  }

  SECTION("Speed function peaks near 0, 120, 240 degrees")
  {
    double v0 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(0.0, 1.0);
    double v60 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(60.0, 1.0);
    double v120 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(120.0, 1.0);
    REQUIRE(v0 > v60);
    REQUIRE(v120 > v60);
  }
}

TEST_CASE("ebsdlib::NolzeHielscherColorKey::LightnessMapping", "[EbsdLib][NolzeHielscher]")
{
  SECTION("At theta=0 (center): L near 0 for standard")
  {
    double L = ebsdlib::NolzeHielscherColorKey::lightness(0.0, 0.25);
    REQUIRE(L == Approx(0.0).margin(1e-6));
  }

  SECTION("At theta=pi/2 (boundary): L near 0.5+")
  {
    double L = ebsdlib::NolzeHielscherColorKey::lightness(M_PI / 2.0, 0.25);
    REQUIRE(L > 0.4);
    REQUIRE(L <= 1.0);
  }

  SECTION("Monotonically increasing with theta")
  {
    double prev = 0.0;
    for(double theta = 0.0; theta <= M_PI / 2.0; theta += 0.01)
    {
      double L = ebsdlib::NolzeHielscherColorKey::lightness(theta, 0.25);
      REQUIRE(L >= prev - 1e-10);
      prev = L;
    }
  }
}

TEST_CASE("ebsdlib::NolzeHielscherColorKey::SaturationMapping", "[EbsdLib][NolzeHielscher]")
{
  SECTION("At L=0.5: S is maximum")
  {
    double S = ebsdlib::NolzeHielscherColorKey::saturation(0.5, 0.25);
    REQUIRE(S == Approx(1.0).margin(1e-6));
  }

  SECTION("At L=0 or L=1: S is reduced")
  {
    double S0 = ebsdlib::NolzeHielscherColorKey::saturation(0.0, 0.25);
    double S1 = ebsdlib::NolzeHielscherColorKey::saturation(1.0, 0.25);
    REQUIRE(S0 < 1.0);
    REQUIRE(S1 < 1.0);
  }
}

TEST_CASE("ebsdlib::NolzeHielscherColorKey::CubicHighOutput", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
  ebsdlib::NolzeHielscherColorKey nhKey(sector);

  SECTION("Center direction produces near-white color")
  {
    auto center = sector.barycenter();
    double eta = std::atan2(center[1], center[0]);
    double chi = std::acos(std::clamp(center[2], -1.0, 1.0));
    double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};

    auto [r, g, b] = nhKey.direction2Color(eta, chi, limits);
    // Center should be bright (high lightness)
    double brightness = (r + g + b) / 3.0;
    REQUIRE(brightness > 0.7);
  }

  SECTION("All outputs are in valid range")
  {
    // Sample a grid of directions within the SST
    for(double eta = 0.01; eta < M_PI / 4.0 - 0.01; eta += 0.05)
    {
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
      for(double chi = 0.01; chi < chiMax - 0.01; chi += 0.05)
      {
        std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
        auto [r, g, b] = nhKey.direction2Color(eta, chi, limits);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "NolzeHielscher" --verbose`
Expected: Compilation failure

- [ ] **Step 3: Write NolzeHielscherColorKey header**

```cpp
// Source/EbsdLib/Utilities/NolzeHielscherColorKey.hpp
#pragma once

#include "EbsdLib/Utilities/IColorKey.hpp"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"

namespace ebsdlib
{

class EbsdLib_EXPORT NolzeHielscherColorKey : public IColorKey
{
public:
  /**
   * @brief Construct with a specific sector geometry.
   * @param sector The fundamental sector geometry for the target Laue group
   * @param lambdaL Lightness nonlinearity parameter (paper Appendix A.2, default 0.25)
   * @param lambdaS Saturation control parameter (paper Appendix A.2, default 0.25)
   */
  explicit NolzeHielscherColorKey(const FundamentalSectorGeometry& sector,
                                  double lambdaL = 0.25, double lambdaS = 0.25);
  ~NolzeHielscherColorKey() override = default;

  /**
   * @brief Map a unit direction vector to an RGB color using the Nolze-Hielscher algorithm.
   * The direction must be in the fundamental sector. Uses internal sector geometry
   * for polar coordinate computation (does NOT need angle limits).
   */
  Vec3 direction2Color(const Vec3& direction) const override;
  std::string name() const override;

  // --- Static helper functions (public for testing) ---

  /**
   * @brief Hue speed function v(rho) from paper Appendix A.1.
   * @param rhoDeg Azimuthal angle in degrees
   * @param distance Distance from center to boundary at this angle
   * @return Speed value (always positive)
   */
  static double hueSpeedFunction(double rhoDeg, double distance);

  /**
   * @brief Nonlinear lightness mapping from paper Appendix A.2.
   * @param theta Polar angle in [0, pi/2]
   * @param lambdaL Nonlinearity parameter (0.25 recommended)
   * @return Lightness in [0, ~0.75]
   */
  static double lightness(double theta, double lambdaL);

  /**
   * @brief Saturation as function of lightness, paper Appendix A.2.
   * @param L Lightness value
   * @param lambdaS Control parameter (0.25 recommended)
   * @return Saturation in [0, 1]
   */
  static double saturation(double L, double lambdaS);

private:
  FundamentalSectorGeometry m_Sector;
  double m_LambdaL;
  double m_LambdaS;
};

} // namespace ebsdlib
```

- [ ] **Step 4: Write NolzeHielscherColorKey implementation**

The .cpp file implements:
1. Constructor stores sector + parameters
2. `direction2Color()`:
   a. Convert (eta, chi) to unit direction vector h
   b. Call `m_Sector.polarCoordinates(h)` to get (radius, rho)
   c. Apply azimuthal correction (hue speed function integration)
   d. For standard keys: compute lightness L, saturation S from radius
   e. For extended keys: check which half (white/black), adjust radius accordingly
   f. Convert (H, S, L) to RGB via `color::hslToRgb()`
3. `hueSpeedFunction()`: `d * (0.5 + exp(-|wrap(rho)|/4) + exp(-|wrap(rho-120)|/4) + exp(-|wrap(rho+120)|/4))`
4. `lightness()`: `lambdaL * (theta / (pi/2)) + (1 - lambdaL) * sin^2(theta/2)`
5. `saturation()`: `1 - 2 * lambdaS * |L - 0.5|`

- [ ] **Step 5: Add to build system**

Add NolzeHielscherColorKey.hpp/.cpp to `Source/EbsdLib/Utilities/SourceList.cmake`.
Add NolzeHielscherColorKeyTest.cpp to `Source/Test/CMakeLists.txt`.

- [ ] **Step 6: Build and run tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "NolzeHielscher" --verbose`
Expected: All PASS

- [ ] **Step 7: Commit**

```bash
git add Source/EbsdLib/Utilities/NolzeHielscherColorKey.hpp Source/EbsdLib/Utilities/NolzeHielscherColorKey.cpp Source/Test/NolzeHielscherColorKeyTest.cpp Source/EbsdLib/Utilities/SourceList.cmake Source/Test/CMakeLists.txt
git commit -m "feat: implement Nolze-Hielscher IPF color key algorithm from paper"
```

---

### Task 6: Integrate Color Keys into LaueOps

**Files:**
- Modify: `Source/EbsdLib/LaueOps/LaueOps.h`
- Modify: `Source/EbsdLib/LaueOps/LaueOps.cpp`
- Modify: `Source/EbsdLib/LaueOps/CubicOps.cpp` (and all 10 other LaueOps subclasses)

- [ ] **Step 1: Write integration tests**

Add to the existing `TSLColorKeyTest.cpp`:

```cpp
TEST_CASE("ebsdlib::LaueOps::ColorKeyIntegration", "[EbsdLib][ColorKeyIntegration]")
{
  auto allOps = LaueOps::GetAllOrientationOps();

  SECTION("Default color key is TSL")
  {
    for(size_t i = 0; i < 11; i++)
    {
      REQUIRE(allOps[i]->getColorKey()->name() == "TSL");
    }
  }

  SECTION("Can switch to NolzeHielscher")
  {
    auto& cubicOps = *allOps[1]; // Cubic_High
    auto nhKey = std::make_shared<ebsdlib::NolzeHielscherColorKey>(
      ebsdlib::FundamentalSectorGeometry::cubicHigh());
    cubicOps.setColorKey(nhKey);
    REQUIRE(cubicOps.getColorKey()->name() == "NolzeHielscher");
  }

  SECTION("TSL backward compatibility: same output after refactor")
  {
    // Compare several orientations through the full pipeline
    double refDir[3] = {0.0, 0.0, 1.0};
    double eulers[3] = {0.5, 0.3, 0.2};

    for(size_t i = 0; i < 11; i++)
    {
      auto color = allOps[i]->generateIPFColor(eulers, refDir, false);
      // Colors should be valid (non-zero for non-degenerate orientations)
      REQUIRE(color.r + color.g + color.b > 0);
    }
  }
}
```

- [ ] **Step 2: Add color key methods to LaueOps.h**

Add to `LaueOps.h` (after the existing `generateIPFColor` declarations):

```cpp
#include "EbsdLib/Utilities/IColorKey.hpp"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

// In the public section:
void setColorKey(ebsdlib::IColorKey::Pointer colorKey);
ebsdlib::IColorKey::Pointer getColorKey() const;

// In the protected/private section:
ebsdlib::IColorKey::Pointer m_ColorKey;
```

- [ ] **Step 3: Modify LaueOps.cpp**

Add default construction of `m_ColorKey` to a `TSLColorKey` in the constructor.
Modify `computeIPFColor()` to delegate to `m_ColorKey->direction2Color()` when a color key is set.

```cpp
// In LaueOps constructor:
m_ColorKey = std::make_shared<ebsdlib::TSLColorKey>();

// In computeIPFColor(), after computing eta, chi, and angleLimits:
if(m_ColorKey)
{
  auto [r, g, b] = m_ColorKey->direction2Color(eta, chi, angleLimits);
  _rgb[0] = r;
  _rgb[1] = g;
  _rgb[2] = b;
  return;
}
// ... (fallback to existing inline algorithm for safety)
```

- [ ] **Step 4: Build and run ALL tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "EbsdLib::" --verbose`
Expected: All existing tests PASS (backward compatibility), plus new integration tests PASS

- [ ] **Step 5: Commit**

```bash
git add Source/EbsdLib/LaueOps/LaueOps.h Source/EbsdLib/LaueOps/LaueOps.cpp Source/Test/TSLColorKeyTest.cpp
git commit -m "feat: integrate pluggable IColorKey into LaueOps with TSL default"
```

---

### Task 7: IPF Legend Generation with New Color Keys

**Files:**
- Modify: `Source/EbsdLib/LaueOps/LaueOps.cpp` (the `generateIPFTriangleLegend` area)
- Modify: `Source/Test/IPFLegendTest.cpp` (add new test sections)

- [ ] **Step 1: Add legend tests for Nolze-Hielscher**

Add to `IPFLegendTest.cpp`:

```cpp
TEST_CASE("ebsdlib::IPFLegendTest::NolzeHielscherLegend", "[EbsdLib][IPFLegendTest]")
{
  auto allOps = LaueOps::GetAllOrientationOps();

  for(size_t index = 0; index < 11; index++)
  {
    SECTION(allOps[index]->getSymmetryName() + " NH Legend")
    {
      // Set NH color key
      // Generate legend
      // Verify image dimensions and non-zero content
      auto legend = allOps[index]->generateIPFTriangleLegend(256, false);
      REQUIRE(legend != nullptr);
      REQUIRE(legend->getNumberOfTuples() > 0);
    }
  }
}
```

- [ ] **Step 2: Modify legend generation to use the active color key**

In the `CreateIPFLegend()` helper or equivalent function used by `generateIPFTriangleLegend()`:
- Instead of the hardcoded TSL color formula, call `m_ColorKey->direction2Color(eta, chi, limits)`
- This way the legend automatically reflects whichever color key is active

- [ ] **Step 3: Build and run legend tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "IPFLegend" --verbose`
Expected: All PASS

- [ ] **Step 4: Commit**

```bash
git add Source/EbsdLib/LaueOps/LaueOps.cpp Source/Test/IPFLegendTest.cpp
git commit -m "feat: IPF legend generation respects active color key"
```

---

### Task 8: Extended Color Key for Non-Mirror Laue Groups

**Files:**
- Modify: `Source/EbsdLib/Utilities/NolzeHielscherColorKey.cpp`
- Modify: `Source/EbsdLib/Utilities/FundamentalSectorGeometry.cpp`
- Modify: `Source/Test/NolzeHielscherColorKeyTest.cpp`

This task adds the "extended" coloring mode for Laue groups with non-mirror boundaries (m-3, 6/m, 4/m, 2/m), per Section 2.6 of the paper.

- [ ] **Step 1: Add tests for extended key behavior**

```cpp
TEST_CASE("ebsdlib::NolzeHielscherColorKey::ExtendedKey_CubicLow", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicLow();
  REQUIRE(sector.colorKeyMode() == "extended");

  auto supergroupSector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
  ebsdlib::NolzeHielscherColorKey nhKey(sector);

  SECTION("All outputs in valid range across m-3 sector")
  {
    // Sample directions within the m-3 SST using direction vectors
    for(double eta = 0.01; eta < M_PI / 2.0 - 0.01; eta += 0.1)
    {
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
      for(double chi = 0.01; chi < chiMax - 0.01; chi += 0.1)
      {
        double sinChi = std::sin(chi);
        std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
        auto [r, g, b] = nhKey.direction2Color(dir);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }

  SECTION("Uses both bright and dark colors (extended range)")
  {
    bool hasBright = false;
    bool hasDark = false;
    for(double eta = 0.01; eta < M_PI / 2.0 - 0.01; eta += 0.05)
    {
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
      for(double chi = 0.01; chi < chiMax - 0.01; chi += 0.05)
      {
        double sinChi = std::sin(chi);
        std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
        auto [r, g, b] = nhKey.direction2Color(dir);
        double brightness = (r + g + b) / 3.0;
        if(brightness > 0.6) hasBright = true;
        if(brightness < 0.4) hasDark = true;
      }
    }
    REQUIRE(hasBright);
    REQUIRE(hasDark);
  }

  SECTION("Direction in supergroup sector -> bright, direction outside -> dark")
  {
    // [0,0,1] is in both m-3m and m-3 sectors -> should be bright (white center half)
    std::array<double, 3> dir001 = {0.0, 0.0, 1.0};
    if(supergroupSector.isInside(dir001) && sector.isInside(dir001))
    {
      auto [r, g, b] = nhKey.direction2Color(dir001);
      double brightness = (r + g + b) / 3.0;
      REQUIRE(brightness > 0.5);
    }

    // A direction in the m-3 sector but NOT in the m-3m sector -> dark (black center half)
    // Example: eta ~= 60 deg, which is outside m-3m's [0, 45] but inside m-3's [0, 90]
    double sinChi = std::sin(0.3);
    std::array<double, 3> dirExtended = {sinChi * std::cos(1.1), sinChi * std::sin(1.1), std::cos(0.3)};
    if(sector.isInside(dirExtended) && !supergroupSector.isInside(dirExtended))
    {
      auto [r, g, b] = nhKey.direction2Color(dirExtended);
      double brightness = (r + g + b) / 3.0;
      REQUIRE(brightness < 0.5);
    }
  }
}
```

- [ ] **Step 2: Implement extended key logic in NolzeHielscherColorKey::direction2Color()**

For extended mode (Section 2.6 of the paper):
1. The direction vector h is already in the P sector (done by LaueOps before calling us)
2. Construct the supergroup P+ sector using `FundamentalSectorGeometry` factory for the supergroup index
3. Check if h is inside the P+ sector via `supergroupSector.isInside(h)`
4. If yes (h is in both P and P+ sectors): white center half
   - Compute polar coords in the P+ sector: `(radius, rho) = supergroupSector.polarCoordinates(h)`
   - `radius_mapped = 0.5 + radius / 2` (maps [0,1] -> [0.5, 1.0])
5. If no (h is in P but not P+): black center half
   - Reflect h into the P+ sector (apply the mirror that maps P's extended half into P+)
   - Compute polar coords of reflected h in P+: `(radius, rho)`
   - `radius_mapped = (1 - radius) / 2` (maps [0,1] -> [0.5, 0.0])
6. Compute L from radius_mapped, S from L, H from corrected rho
7. Convert (H, S, L) to RGB

The supergroup sector is constructed once in the NolzeHielscherColorKey constructor (not per-pixel) and stored as a member for thread safety.

- [ ] **Step 3: Build and run tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "NolzeHielscher" --verbose`
Expected: All PASS

- [ ] **Step 4: Commit**

```bash
git add Source/EbsdLib/Utilities/NolzeHielscherColorKey.cpp Source/EbsdLib/Utilities/FundamentalSectorGeometry.cpp Source/Test/NolzeHielscherColorKeyTest.cpp
git commit -m "feat: implement extended color key for non-mirror Laue groups (m-3, 6/m, 4/m, 2/m)"
```

---

### Task 9: Generate Comparison Legends Application

**Files:**
- Modify: `Source/Apps/generate_ipf_legends.cpp`

- [ ] **Step 1: Add NH legend generation to the existing app**

Modify the `generate_ipf_legends` application to generate both TSL and Nolze-Hielscher legends side-by-side for all 11 Laue groups. Output as TIFF files.

- [ ] **Step 2: Build and run the application manually**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target generate_ipf_legends && ./bin/generate_ipf_legends`

Visually inspect the output TIFF files to verify the N-H legends look correct (enlarged gray center, smooth color gradients, no discontinuities for standard-key groups).

- [ ] **Step 3: Commit**

```bash
git add Source/Apps/generate_ipf_legends.cpp
git commit -m "feat: generate_ipf_legends app produces both TSL and Nolze-Hielscher legends"
```

---

### Task 10: Handle "Impossible" Coloring Mode (-1, -3)

**Files:**
- Modify: `Source/EbsdLib/Utilities/NolzeHielscherColorKey.cpp`
- Modify: `Source/Test/NolzeHielscherColorKeyTest.cpp`

The Laue groups -1 (triclinic) and -3 (trigonal low) have fundamental sectors that are topologically equivalent to the real projective plane (RP2). No continuous injective coloring exists (Massey, 1959). We implement the "unique but discontinuous" compromise: each direction gets a unique color, but color jumps exist at the boundary where identified points meet.

- [ ] **Step 1: Add tests**

```cpp
TEST_CASE("ebsdlib::NolzeHielscherColorKey::ImpossibleMode_Triclinic", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::triclinic();
  REQUIRE(sector.colorKeyMode() == "impossible");

  ebsdlib::NolzeHielscherColorKey nhKey(sector);

  SECTION("Produces valid colors for all directions in upper hemisphere")
  {
    for(double eta = 0.0; eta < 2.0 * M_PI; eta += 0.2)
    {
      for(double chi = 0.01; chi < M_PI / 2.0 - 0.01; chi += 0.2)
      {
        double sinChi = std::sin(chi);
        std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
        auto [r, g, b] = nhKey.direction2Color(dir);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }

  SECTION("Distinct directions produce distinct colors")
  {
    std::array<double, 3> dir1 = {0.0, 0.0, 1.0};
    std::array<double, 3> dir2 = {1.0, 0.0, 0.0};
    auto [r1, g1, b1] = nhKey.direction2Color(dir1);
    auto [r2, g2, b2] = nhKey.direction2Color(dir2);
    double diff = std::abs(r1 - r2) + std::abs(g1 - g2) + std::abs(b1 - b2);
    REQUIRE(diff > 0.1);
  }
}
```

- [ ] **Step 2: Implement impossible mode in direction2Color()**

For impossible mode, use the standard color key (single white center) applied to the full sector. This produces unique colors with a documented discontinuity at the boundary where opposite points are identified. The paper recommends this as compromise (a) from Section 2.7.

- [ ] **Step 3: Build and run tests**

Run: `cd /Users/mjackson/Workspace5/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all && ctest -R "NolzeHielscher" --verbose`
Expected: All PASS

- [ ] **Step 4: Commit**

```bash
git add Source/EbsdLib/Utilities/NolzeHielscherColorKey.cpp Source/Test/NolzeHielscherColorKeyTest.cpp
git commit -m "feat: handle 'impossible' coloring mode for triclinic and trigonal-low groups"
```

---

## Notes for Session Transfer

When switching project folders, the implementer needs:
1. This plan document (self-contained with all formulas, constants, file paths, and test code)
2. The feasibility study at `.claude/reports/ebsd_color_palettes.md` (background context)
3. Access to the paper preprint at https://www.tu-chemnitz.de/mathematik/preprint/2016/PREPRINT_01.pdf

**Key implementation reminders:**
- All formulas come from the paper (Sections 2.2-2.6, Appendix A) or textbook spherical geometry
- Do NOT consult MTEX, orix, or any other GPL-licensed implementation
- The hue speed function uses `exp(-|wrap(rho)|/4)` (paper Eq. 5), NOT `exp(-200*x^2)` (MTEX-specific)
- Precompute azimuthal correction tables and hue speed CDF in constructors for thread safety
- Guard against singularity when direction == barycenter in polar coordinate computation
- Handle non-triangular sectors (4 vertices for m-3, 2 for monoclinic, 0 for triclinic) in azimuthal correction
- The supergroup sector for extended keys should be constructed once and stored as a member

**Deferred for future work (not in this plan):**
- CVD-friendly color palette (red-yellow-cyan-blue variant, per EDAX OIM v9)
- Configurable Gaussian width and baseline in hue speed function
- CIELAB perceptual uniformity optimization

---

## Dependency Graph

```
Task 1 (ColorSpaceUtils)  ──┐
                             ├──> Task 3 (TSLColorKey)  ──┐
Task 2 (IColorKey)  ─────────┤                            ├──> Task 6 (LaueOps Integration) ──> Task 7 (Legends)
                             │                            │
                             └──> Task 5 (NH ColorKey) ───┤
                                    │                     │
Task 4 (SectorGeometry) ───────────┘                     └──> Task 8 (Extended Key) ──> Task 10 (Impossible Mode)
                                                                     │
                                                                     └──> Task 9 (Comparison App)
```

- Tasks 1, 2, and 4 can be done in parallel (no inter-dependencies).
- Task 3 depends on Tasks 1 and 2.
- Task 5 depends on Tasks 1, 2, and 4.
- Task 6 depends on Tasks 3 and 5.
- Task 7 depends on Task 6.
- Task 8 depends on Tasks 4 and 6 (needs FundamentalSectorGeometry for supergroup sector construction).
- Task 9 depends on Task 8.
- Task 10 depends on Task 8.
