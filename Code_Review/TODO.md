# EbsdLib Unit Testing TODO

## Overview

This document catalogs all testable classes/modules in the EbsdLib library, documents existing test coverage, identifies gaps, and prioritizes what needs new or improved unit tests.

**Date:** 2026-02-20
**Test Framework:** Catch2 (with legacy `DREAM3D_REQUIRE*` macros in `UnitTestSupport.hpp`)
**Test Directory:** `Source/Test/`

---

## Existing Test Coverage Summary

There are **12 active test files** compiled into a single `EbsdLibUnitTest` executable. Coverage ranges from comprehensive (OrientationTest) to smoke-test-only (ODFTest, TextureTest, IPFLegendTest).

| Test File | Classes Tested | Coverage Level |
|-----------|---------------|----------------|
| `OrientationTest.cpp` | All 8 Orientation types (Euler, OrientationMatrix, AxisAngle, Rodrigues, Quaternion, Homochoric, Cubochoric, Stereographic) | **Comprehensive** - 4913 test orientations, 42 round-trip conversion paths per type |
| `ConvertToFundamentalZoneTest.cpp` | All 11 LaueOps, Rodrigues, Quaternion | **Comprehensive** - 22 Rodrigues vectors tested against all 11 symmetry groups |
| `QuaternionTest.cpp` | Quaternion, Matrix3X1 (cosTheta), Matrix3X3 (Eigen interop) | **Good** - tests algebraic properties, passive/active rotations |
| `CtfReaderTest.cpp` | CtfReader, CtfPhase | **Good** - European/US locales, multi-phase, error cases, round-trip write |
| `H5EspritReaderTest.cpp` | H5EspritReader | **Good** - error paths, header values, data types, selective array reading |
| `AngImportTest.cpp` | AngReader | **Moderate** - normal read + several malformed file scenarios |
| `EdaxOIMReaderTest.cpp` | H5OIMReader | **Basic** - verifies data arrays load (non-null pointers only) |
| `AngleFileLoaderTest.cpp` | AngleFileLoader | **Basic** - 4 delimiter variants, no value verification |
| `OrientationConverterTest.cpp` | OrientationConverter hierarchy (6 types) | **Moderate but buggy** - has tolerance bug (see Bugs section) |
| `IPFLegendTest.cpp` | LaueOps IPF generation, TiffWriter | **Smoke test** - only checks no crash |
| `ODFTest.cpp` | CubicOps, Texture, Euler, Matrix3X3 | **Smoke test** - has TODO for real comparisons |
| `TextureTest.cpp` | Texture, StatsGen, Matrix3X3, all LaueOps | **Smoke test** - no output validation |

**Not_Used/ directory** (excluded from build): `H5OINAReaderTest.cpp`, `OrientationTransformsTest.cpp`, `SO3SamplerTest.cpp`

---

## Bugs Found During Code Review

### Bug 1: `Matrix3X1::sortAscending()` - Assigns pointer instead of dereferencing

**File:** `Source/EbsdLib/Math/Matrix3X1.hpp:198`
**Severity:** HIGH - Will not compile or will produce wrong results
**Issue:** `SelfType outMat = this;` assigns the raw pointer `this` instead of dereferencing it. Should be `SelfType outMat = *this;`. The function also appears to be missing a `return outMat;` statement at the end.

### Bug 2: `Matrix3X1::normalize()` (static) - Missing return statement

**File:** `Source/EbsdLib/Math/Matrix3X1.hpp:190`
**Severity:** HIGH - Undefined behavior
**Issue:** The static `normalize(T& i, T& j, T& k)` method is declared as returning `bool` but has no `return true;` on the success path (only `return false;` on the zero-denominator path). This is undefined behavior in C++.

### Bug 3: `Stereographic::isValid()` - Wrong epsilon constant

**File:** `Source/EbsdLib/Orientation/Stereographic.hpp:88`
**Severity:** HIGH - Validation is effectively disabled
**Issue:** `value_type epsd = 1.0E15;` should be `1.0E-15`. The current value (one quadrillion) means the validity check `rd > 1.0 + epsd` will never trigger, making `isValid()` always return success regardless of the actual magnitude.

### Bug 4: `OrientationConverterTest.cpp` - Wrong tolerance in test assertions

**File:** `Source/Test/OrientationConverterTest.cpp` (lines ~86 and ~145)
**Severity:** MEDIUM - Tests pass even with wildly incorrect values
**Issue:** The tolerance check uses `delta < 1.0E6` (one million) instead of `1.0E-6` (one millionth). This means the round-trip conversion tests will pass even if values are off by up to a million, making the tests effectively useless for catching conversion errors.

### Bug 5: `EbsdDataArray::eraseTuples()` - Does not update `m_NumTuples`

**File:** `Source/EbsdLib/Core/EbsdDataArray.cpp:678`
**Severity:** MEDIUM - `getNumberOfTuples()` returns stale value after eraseTuples
**Issue:** The `eraseTuples()` method updates `m_Size`, `m_MaxId`, and `m_Array` but never updates `m_NumTuples`. After calling `eraseTuples()`, `getNumberOfTuples()` returns the original tuple count instead of the new (reduced) count. `getSize()` correctly returns the new total element count.

---

## Classes Requiring Unit Tests

### HIGH Priority (No tests + significant functionality)

These classes have zero or near-zero test coverage and contain non-trivial logic that is widely used in the library.

#### 1. `EbsdDataArray<T>` - Core data container

- **File:** `Source/EbsdLib/Core/EbsdDataArray.hpp`
- **Status:** **DONE** - `Source/Test/EbsdDataArrayTest.cpp` (23 test cases)
- **Why:** Core template class used by virtually every reader and computation. Wraps raw arrays with lifecycle management.
- **Recommended tests:**
  - Construction (default, sized, from existing pointer)
  - `resize()`, `resizeTuples()`
  - Element access (`setValue`, `getValue`, `operator[]`, `getPointer`)
  - `copyTuples()`, `eraseTuples()`
  - `deepCopy()`
  - Component-based access (`getComponent`, `setComponent`, `getNumberOfComponents`)
  - `initializeWithZeros()`, `initializeWithValue()`
  - Edge cases: zero-size allocation, out-of-bounds access behavior

#### 2. `Matrix3X1<T>` - 3x1 vector operations

- **File:** `Source/EbsdLib/Math/Matrix3X1.hpp`
- **Status:** **DONE** - `Source/Test/Matrix3X1Test.cpp` (20 test cases)
- **Why:** Core math class with known bugs (see Bugs #1 and #2)
- **Recommended tests:**
  - Construction and element access
  - `dot()`, `cross()`, `magnitude()`
  - `normalize()` (member and static versions) - **must catch Bug #2**
  - `sortAscending()` - **must catch Bug #1**
  - `cosTheta()` (expand existing tests)
  - Operator overloads (`+`, `-`, unary `-`)
  - Edge cases: zero vector, unit vector, near-zero magnitude

#### 3. `Matrix3X3<T>` - 3x3 matrix operations

- **File:** `Source/EbsdLib/Math/Matrix3X3.hpp`
- **Status:** **DONE** - `Source/Test/Matrix3X3Test.cpp` (23 test cases)
- **Why:** Used in orientation math, coordinate transforms; currently only smoke-tested
- **Recommended tests:**
  - Construction and element access
  - `multiply()` (matrix-matrix, matrix-vector)
  - `transpose()`, `invert()`, `adjoint()`
  - `determinant()`, `cofactor()`, `minors()`
  - `normalize()`, `identity()`
  - Known results: `A * A^-1 = I`, `det(A^T) = det(A)`, `(AB)^T = B^T * A^T`

#### 4. `CprReader` - Oxford binary file reader

- **File:** `Source/EbsdLib/IO/HKL/CprReader.h`
- **Status:** No tests
- **Why:** Complex binary parser with zero coverage; parses .cpr/.crc file pairs
- **Recommended tests:**
  - Read valid `.cpr`/`.crc` files, verify header and data values
  - Error handling: missing file, corrupted header, truncated data
  - Multi-phase files
  - Comparison with equivalent `.ctf` data for the same sample

#### 5. `H5OINAReader` - Oxford HDF5 reader

- **File:** `Source/EbsdLib/IO/HKL/H5OINAReader.h`
- **Status:** Test exists in Not_Used/ directory but is not compiled
- **Why:** Reader for an important format with no active tests
- **Recommended tests:**
  - Restore and update the existing `Not_Used/H5OINAReaderTest.cpp`
  - Read valid OINA files, verify headers and data arrays
  - Error handling: missing file, bad HDF5 path

#### 6. `SO3Sampler` - Rotation space sampling

- **File:** `Source/EbsdLib/LaueOps/SO3Sampler.h`
- **Status:** Test exists in Not_Used/ directory but is not compiled
- **Why:** Sampling in SO(3) is mathematically non-trivial
- **Recommended tests:**
  - Restore and update the existing `Not_Used/SO3SamplerTest.cpp`
  - Verify sample count for given resolution
  - Verify all generated samples are valid orientations
  - Verify approximate uniformity of distribution
  - Test with each Laue group's fundamental zone

#### 7. `OrientationMath` - Crystallographic math

- **File:** `Source/EbsdLib/Core/OrientationMath.h`
- **Status:** **DONE** - `Source/Test/OrientationMathTest.cpp` (12 test cases)
- **Why:** Static methods for misorientation calculations, widely used
- **Recommended tests:**
  - `axisAngletoMatrix()`, `quatsToMatrix()`
  - `getMatSymOp()` for each crystal symmetry
  - Misorientation calculation between known orientations
  - Edge cases: identity quaternion, 180-degree rotations

#### 8. `EbsdStringUtils` - String utilities

- **File:** `Source/EbsdLib/Utilities/EbsdStringUtils.hpp`
- **Status:** **DONE** - `Source/Test/EbsdStringUtilsTest.cpp` (18 test cases)
- **Why:** String parsing utilities used by all readers
- **Recommended tests:**
  - `split()`, `tokenize()` with various delimiters
  - `trimLeft()`, `trimRight()`, `trim()`
  - `replace()`, `toUpper()`, `toLower()`
  - Number conversions (`toInt()`, `toFloat()`, `toDouble()`)
  - Edge cases: empty strings, no delimiter found, multi-character delimiters

#### 9. `EbsdTransform` - Reference frame transformations

- **File:** `Source/EbsdLib/Core/EbsdTransform.h`
- **Status:** **DONE** - `Source/Test/EbsdTransformTest.cpp` (6 test cases)
- **Why:** Transforms sample and Euler reference frames; errors here corrupt all downstream analysis
- **Recommended tests:**
  - Sample reference frame transformations (all axis combinations)
  - Euler reference frame transformations
  - Identity transformation (no change)
  - Round-trip transformation consistency

#### 10. `EbsdLibRandom` - PRNG

- **File:** `Source/EbsdLib/Math/EbsdLibRandom.h`
- **Status:** **DONE** - `Source/Test/EbsdLibRandomTest.cpp` (9 test cases)
- **Why:** Mersenne Twister wrapper; used for texture generation
- **Recommended tests:**
  - Seeded deterministic output verification
  - Range verification (`genrand_real1()` in [0,1], `genrand_res53()` precision)
  - `genrand_int32()` produces valid values

#### 11. `ArrayHelpers<T,K>` - Template math helpers

- **File:** `Source/EbsdLib/Math/ArrayHelpers.hpp`
- **Status:** **DONE** - `Source/Test/ArrayHelpersTest.cpp` (13 test cases)
- **Why:** Static utility methods used in orientation conversions
- **Recommended tests:**
  - `splat()`, `multiply()`, `scalarMultiply()`
  - `sum()`, `normalize()`, `maxval()`, `minval()`
  - `sumofSquares()`, `copyElements()`

#### 12. `DataParser` - HKL column data parsing

- **File:** `Source/EbsdLib/IO/HKL/DataParser.hpp`
- **Status:** No tests
- **Why:** Template hierarchy for parsing typed columns from text files
- **Recommended tests:**
  - Parse int, float, double columns
  - Malformed data handling
  - Empty/missing values

---

### MEDIUM Priority (Partial tests or less critical)

#### 13. `ColorTable` / `ColorUtilities` - Color handling

- **Files:** `Source/EbsdLib/Utilities/ColorTable.h`, `Source/EbsdLib/Utilities/ColorUtilities.h`
- **Status:** **DONE** - `Source/Test/ColorTableTest.cpp` (11 test cases)
- **Recommended tests:** `RgbColor` helpers, HSV-to-RGB conversion, color component extraction

#### 14. `LambertUtilities` - Square-to-sphere mapping

- **File:** `Source/EbsdLib/Utilities/LambertUtilities.h`
- **Status:** **DONE** - `Source/Test/LambertUtilitiesTest.cpp` (4 test cases)
- **Recommended tests:** Square-to-sphere and sphere-to-square conversions, round-trip consistency, boundary values

#### 15. `ModifiedLambertProjection` - Lambert projection

- **File:** `Source/EbsdLib/Utilities/ModifiedLambertProjection.h`
- **Status:** **DONE** - `Source/Test/ModifiedLambertProjectionTest.cpp` (7 test cases)
- **Recommended tests:** North/south hemisphere projection, `addInterpolatedValues()`, normalization

#### 16. `ComputeStereographicProjection` - Stereographic projection utilities

- **File:** `Source/EbsdLib/Utilities/ComputeStereographicProjection.h`
- **Status:** **DONE** - `Source/Test/StereographicProjectionTest.cpp` (6 test cases)
- **Recommended tests:** Stereographic-to-spherical and spherical-to-stereographic round-trips

#### 17. `TexturePreset` - Texture presets

- **File:** `Source/EbsdLib/Texture/TexturePreset.h`
- **Status:** **DONE** - `Source/Test/TexturePresetTest.cpp` (5 test cases)
- **Recommended tests:** Preset value getters, preset registration

#### 18. `AngPhase` / `CtfPhase` / `EspritPhase` - Phase data classes

- **Files:** `Source/EbsdLib/IO/TSL/AngPhase.h`, `Source/EbsdLib/IO/HKL/CtfPhase.h`, `Source/EbsdLib/IO/BrukerNano/EspritPhase.h`
- **Status:** **DONE** - `Source/Test/PhaseTest.cpp` (18 test cases)
- **Recommended tests:** Construction, getter/setter verification, lattice constant parsing

#### 19. LaueOps subclasses - Enhanced symmetry tests

- **Files:** `Source/EbsdLib/LaueOps/*.h` (11 subclasses)
- **Status:** **DONE** - `Source/Test/LaueOpsTest.cpp` (11 test cases)
- **Recommended tests:**
  - `getNumSymOps()` returns expected count for each crystal system
  - `getIPFColor()` with known orientations against reference values
  - `getMisorientationColor()` with known misorientations
  - `calculateMisorientation()` between known orientations

#### 20. `ModifiedLambertProjection3D<T,K>` - 3D Lambert projection

- **File:** `Source/EbsdLib/Utilities/ModifiedLambertProjection3D.hpp`
- **Status:** **DONE** - `Source/Test/ModifiedLambertProjection3DTest.cpp` (7 test cases)
- **Recommended tests:** Cube-to-sphere and sphere-to-cube conversions, edge cases at cube boundaries

#### 21. `OrientationTransformation` (namespace) - Conversion functions

- **File:** `Source/EbsdLib/Core/OrientationTransformation.hpp`
- **Status:** **DONE** - `Source/Test/OrientationTransformationTest.cpp` (11 test cases)
- **Recommended tests:** Direct tests of each `xx2yy()` function with known analytical values (supplement existing round-trip tests)

#### 22. `H5CtfReader` / `H5AngReader` - HDF5 format readers

- **Files:** `Source/EbsdLib/IO/HKL/H5CtfReader.h`, `Source/EbsdLib/IO/TSL/H5AngReader.h`
- **Status:** No tests (H5OIMReader is tested but these are not)
- **Recommended tests:** Read valid HDF5 files, verify data arrays, error handling

---

### LOW Priority (Simple utilities / interfaces / data holders)

#### 23. `ToolTipGenerator` - HTML tooltip builder

- **File:** `Source/EbsdLib/Utilities/ToolTipGenerator.h`
- **Status:** **DONE** - `Source/Test/ToolTipGeneratorTest.cpp` (8 test cases). Also fixed bug where `generateHTML()` and `rowToHTML()` returned empty strings.
- **Recommended tests:** `addTitle()`, `addValue()`, output HTML correctness

#### 24. `PoleFigureData` - Data holder

- **File:** `Source/EbsdLib/Utilities/PoleFigureData.h`
- **Status:** **DONE** - `Source/Test/PoleFigureDataTest.cpp` (5 test cases)
- **Recommended tests:** Construction, getter verification

#### 25. `CanvasUtilities` - Visualization helpers

- **File:** `Source/EbsdLib/Utilities/CanvasUtilities.hpp`
- **Status:** **DONE** - `Source/Test/CanvasUtilitiesTest.cpp` (6 test cases)
- **Recommended tests:** Only if visual regression testing infrastructure is added

#### 26. `ModifiedLambertProjectionArray` - Array variant

- **File:** `Source/EbsdLib/Utilities/ModifiedLambertProjectionArray.h`
- **Status:** **DONE** - `Source/Test/ModifiedLambertProjectionArrayTest.cpp` (16 test cases)
- **Recommended tests:** Array construction, element access, resize

#### 27. `PoleFigureUtilities` - Pole figure generation

- **File:** `Source/EbsdLib/Utilities/PoleFigureUtilities.h`
- **Status:** **DONE** - `Source/Test/PoleFigureUtilitiesTest.cpp` (4 test cases)
- **Recommended tests:** Configuration setup, pole figure generation with known inputs

#### 28. `TiffWriter` - TIFF file output

- **File:** `Source/EbsdLib/Utilities/TiffWriter.h`
- **Status:** **DONE** - `Source/Test/TiffWriterTest.cpp` (4 test cases)
- **Recommended tests:** Write and read-back verification, grayscale and color modes

---

## Existing Tests Needing Improvement

These tests exist but have known deficiencies.

| Test File | Issue | Action Needed |
|-----------|-------|---------------|
| `OrientationConverterTest.cpp` | Tolerance `1.0E6` should be `1.0E-6` | **Fix bug**, verify tests still pass |
| `ODFTest.cpp` | Both tests are smoke-only, has explicit TODO | Add value comparisons against known results |
| `TextureTest.cpp` | All 4 tests have no assertions on results | Add value comparisons for Matrix3X3 ops and texture generation |
| `IPFLegendTest.cpp` | Only checks no crash, has explicit TODO | Add golden image comparison or at least pixel value spot-checks |
| `AngleFileLoaderTest.cpp` | Only checks error code == 0 | Add verification of loaded angle values |
| `EdaxOIMReaderTest.cpp` | Only checks non-null pointers | Add spot-checks of actual data values |

---

## Suggested Implementation Order

1. **Fix known bugs first** (Bugs #1-4 above)
2. **Core math classes** - Matrix3X1, Matrix3X3, ArrayHelpers, EbsdLibMath (foundation for everything else)
3. **Core data container** - EbsdDataArray (used everywhere)
4. **String/utility classes** - EbsdStringUtils, EbsdTransform (used by readers)
5. **Improve existing smoke tests** - ODFTest, TextureTest, IPFLegendTest, OrientationConverterTest
6. **IO readers with no coverage** - CprReader, H5OINAReader, H5CtfReader, H5AngReader
7. **OrientationMath and SO3Sampler** - Crystallographic math
8. **Lambert/projection utilities** - LambertUtilities, ModifiedLambertProjection, ComputeStereographicProjection
9. **LaueOps enhanced tests** - IPF coloring, misorientation calculation
10. **Remaining utilities** - ColorTable, TexturePreset, phase classes, low-priority items

---

## Notes

- All tests use Catch2 and are compiled into a single executable (`EbsdLibUnitTest`)
- Tests requiring HDF5 data files are guarded by `#ifdef EbsdLib_ENABLE_HDF5`
- Test data files are managed via CMake and extracted at test time by `ctest`
- The `Not_Used/` directory contains 3 test files that could be restored and updated: `H5OINAReaderTest.cpp`, `SO3SamplerTest.cpp`, `OrientationTransformsTest.cpp`
- Consider migrating from legacy `DREAM3D_REQUIRE*` macros to native Catch2 `REQUIRE`/`CHECK` assertions in new tests
