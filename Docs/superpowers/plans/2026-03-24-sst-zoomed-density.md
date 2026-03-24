# SST-Zoomed IPF Density Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make IPF density images fill the frame by mapping pixels to only the SST bounding box (eta/chi range) instead of the full Lambert hemisphere disk.

**Architecture:** Add an optional `sstBoundingBox` parameter to `computeIPFIntensity()`. When provided, pixels map to the SST region in (eta, chi) spherical coordinates. The bounding box values come from the existing `getIpfColorAngleLimits()` virtual method already on all 11 LaueOps subclasses. `generateAnnotatedIPFDensity()` passes the bounding box automatically.

**Tech Stack:** C++20, EbsdLib LaueOps, InversePoleFigureUtilities, Lambert projection

---

## File Map

| File | Change |
|------|--------|
| `Source/EbsdLib/Utilities/InversePoleFigureUtilities.h` | Add optional `sstBoundingBox` parameter to `computeIPFIntensity()` |
| `Source/EbsdLib/Utilities/InversePoleFigureUtilities.cpp` | Implement SST-zoomed pixel mapping when bounding box provided |
| `Source/EbsdLib/LaueOps/LaueOps.cpp` | Update `generateAnnotatedIPFDensity()` to compute bounding box from `getIpfColorAngleLimits()` and pass it |

No new files. No changes to any of the 11 subclass files (the existing `getIpfColorAngleLimits()` already provides what we need).

## Key Reference

**Existing `getIpfColorAngleLimits(double eta)`** — virtual method on all 11 LaueOps subclasses. Returns `std::array<double, 3>` = `{etaMin, etaMax, chiMax}` in radians. For cubic classes, `chiMax` varies with `eta`; for all others it's constant (90° = π/2).

**Existing constants per subclass** (all in degrees, in each .cpp file):

| Subclass | etaMin | etaMax | chiMax |
|----------|--------|--------|--------|
| CubicOps | 0 | 45 | dynamic: arccos(1/sqrt(3)) ≈ 54.74° at eta=45° |
| CubicLowOps | 0 | 90 | dynamic: arccos(1/sqrt(3)) ≈ 54.74° at eta=45° |
| HexagonalOps | 0 | 30 | 90 |
| HexagonalLowOps | 0 | 60 | 90 |
| TrigonalOps | -90 | -30 | 90 |
| TrigonalLowOps | -120 | 0 | 90 |
| TetragonalOps | 0 | 45 | 90 |
| TetragonalLowOps | 0 | 90 | 90 |
| OrthoRhombicOps | 0 | 90 | 90 |
| MonoclinicOps | 0 | 180 | 90 |
| TriclinicOps | 0 | 180 | 90 |

Note: TrigonalOps and TrigonalLowOps have **negative** eta ranges. The SST mapping must handle negative eta values correctly.

---

## Tasks

### Task 1: Modify `computeIPFIntensity()` to accept SST bounding box

**Files:**
- Modify: `Source/EbsdLib/Utilities/InversePoleFigureUtilities.h`
- Modify: `Source/EbsdLib/Utilities/InversePoleFigureUtilities.cpp`

- [ ] **Step 1: Update the function signature in the header**

In `InversePoleFigureUtilities.h`, change the declaration from:

```cpp
  static ebsdlib::DoubleArrayType::Pointer computeIPFIntensity(const LaueOps& ops, ebsdlib::FloatArrayType* ipfDirections, int imageWidth, int imageHeight, int lambertDim, bool normalizeMRD);
```

To:

```cpp
  static ebsdlib::DoubleArrayType::Pointer computeIPFIntensity(const LaueOps& ops, ebsdlib::FloatArrayType* ipfDirections, int imageWidth, int imageHeight, int lambertDim, bool normalizeMRD,
                                                                const std::array<double, 4>* sstBoundingBox = nullptr);
```

Add `#include <array>` if not already present.

Update the doc comment to add:
```
   * @param sstBoundingBox Optional SST bounding box {etaMin, etaMax, chiMin, chiMax} in radians.
   *   When provided, pixels map to only this region in (eta, chi) space, making the SST fill the image.
   *   When nullptr, uses the default full Lambert hemisphere disk mapping.
```

- [ ] **Step 2: Update the function signature in the .cpp file**

In `InversePoleFigureUtilities.cpp`, update the function definition to match the new signature:

```cpp
ebsdlib::DoubleArrayType::Pointer InversePoleFigureUtilities::computeIPFIntensity(const LaueOps& ops, ebsdlib::FloatArrayType* ipfDirections, int imageWidth, int imageHeight, int lambertDim,
                                                                                  bool normalizeMRD, const std::array<double, 4>* sstBoundingBox)
```

- [ ] **Step 3: Add the SST-zoomed pixel mapping code path**

In `InversePoleFigureUtilities.cpp`, replace the pixel iteration loop (the "Step 3" section, lines ~172-234) with code that handles both modes. The Lambert binning (Steps 1-2) stays unchanged. Replace from the `// Step 3:` comment through the end of the function:

```cpp
  // Step 3: Create the output intensity image
  std::vector<size_t> tDims = {static_cast<size_t>(imageWidth * imageHeight)};
  std::vector<size_t> cDims = {1};
  ebsdlib::DoubleArrayType::Pointer intensity = ebsdlib::DoubleArrayType::CreateArray(tDims, cDims, "IPF_Intensity", true);
  double* intensityPtr = intensity->getPointer(0);

  if(sstBoundingBox != nullptr)
  {
    // SST-zoomed mode: map pixels to the SST bounding box in (eta, chi) space
    double etaMin = (*sstBoundingBox)[0];
    double etaMax = (*sstBoundingBox)[1];
    double chiMin = (*sstBoundingBox)[2];
    double chiMax = (*sstBoundingBox)[3];

    for(int y = 0; y < imageHeight; y++)
    {
      for(int x = 0; x < imageWidth; x++)
      {
        int index = y * imageWidth + x;

        // Map pixel to (eta, chi) within the bounding box
        // x maps to eta (left=etaMin, right=etaMax)
        // y maps to chi (top=chiMin, bottom=chiMax)
        double eta = etaMin + (static_cast<double>(x) + 0.5) / static_cast<double>(imageWidth) * (etaMax - etaMin);
        double chi = chiMin + (static_cast<double>(y) + 0.5) / static_cast<double>(imageHeight) * (chiMax - chiMin);

        // Check if direction is inside the SST
        if(!ops.inUnitTriangle(eta, chi))
        {
          intensityPtr[index] = -1.0;
          continue;
        }

        // Convert (eta, chi) to unit sphere xyz
        double sinChi = std::sin(chi);
        std::array<float, 3> xyz = {
            static_cast<float>(sinChi * std::cos(eta)),
            static_cast<float>(sinChi * std::sin(eta)),
            static_cast<float>(std::cos(chi))};

        // Look up the interpolated intensity from the Lambert projection
        std::array<float, 2> sqCoord = {0.0f, 0.0f};
        bool isNorth = lambert->getSquareCoord(xyz.data(), sqCoord.data());
        if(isNorth)
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::NorthSquare, sqCoord.data());
        }
        else
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::SouthSquare, sqCoord.data());
        }
      }
    }
  }
  else
  {
    // Original full-disk mode: Lambert azimuthal equal-area projection
    float unitRadius = std::sqrt(2.0f);
    float span = 2.0f * unitRadius;
    float xres = span / static_cast<float>(imageWidth);
    float yres = span / static_cast<float>(imageHeight);

    int halfWidth = imageWidth / 2;
    int halfHeight = imageHeight / 2;

    for(int y = 0; y < imageHeight; y++)
    {
      for(int x = 0; x < imageWidth; x++)
      {
        int index = y * imageWidth + x;

        float xtmp = static_cast<float>(x - halfWidth) * xres + (xres * 0.5f);
        float ytmp = static_cast<float>(y - halfHeight) * yres + (yres * 0.5f);

        float rhoSq = xtmp * xtmp + ytmp * ytmp;

        if(rhoSq > 2.0f)
        {
          intensityPtr[index] = -1.0;
          continue;
        }

        float t = std::sqrt(1.0f - rhoSq / 4.0f);
        std::array<float, 3> xyz = {xtmp * t, ytmp * t, 1.0f - rhoSq / 2.0f};

        double chi = std::acos(static_cast<double>(xyz[2]));
        double eta = std::atan2(static_cast<double>(xyz[1]), static_cast<double>(xyz[0]));

        if(!ops.inUnitTriangle(eta, chi))
        {
          intensityPtr[index] = -1.0;
          continue;
        }

        std::array<float, 2> sqCoord = {0.0f, 0.0f};
        bool isNorth = lambert->getSquareCoord(xyz.data(), sqCoord.data());
        if(isNorth)
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::NorthSquare, sqCoord.data());
        }
        else
        {
          intensityPtr[index] = lambert->getInterpolatedValue(ModifiedLambertProjection::SouthSquare, sqCoord.data());
        }
      }
    }
  }

  return intensity;
```

- [ ] **Step 4: Build the library**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target EbsdLib 2>&1 | tail -10
```
Expected: Clean build. The default parameter means all existing callers still work unchanged.

- [ ] **Step 5: Commit**

```bash
cd /Users/mjackson/Workspace1/EbsdLib
git add Source/EbsdLib/Utilities/InversePoleFigureUtilities.h Source/EbsdLib/Utilities/InversePoleFigureUtilities.cpp
git commit -m "ENH: Add SST bounding box parameter to computeIPFIntensity for zoomed density"
```

---

### Task 2: Update `generateAnnotatedIPFDensity()` to pass the bounding box

**Files:**
- Modify: `Source/EbsdLib/LaueOps/LaueOps.cpp`

- [ ] **Step 1: Compute the SST bounding box and pass it to `computeIPFIntensity()`**

In `LaueOps.cpp`, in the `generateAnnotatedIPFDensity()` method, find the three calls to `computeIPFIntensity()` (around line 1154-1156). Before those calls, add code to compute the bounding box from the existing `getIpfColorAngleLimits()` method:

```cpp
  // Compute SST bounding box for zoomed density images
  // getIpfColorAngleLimits returns {etaMin, etaMax, chiMax(eta)} in radians.
  // For cubic classes, chiMax varies with eta; use max value at etaMax.
  auto angleLimits = getIpfColorAngleLimits(0.0); // Get etaMin, etaMax
  double etaMin = angleLimits[0];
  double etaMax = angleLimits[1];

  // Get chiMax at etaMax (gives the largest chiMax for cubic; same for others)
  auto angleLimitsAtMax = getIpfColorAngleLimits(etaMax);
  double chiMax = angleLimitsAtMax[2];

  // Also check chiMax at etaMin in case it's larger (shouldn't be, but safe)
  auto angleLimitsAtMin = getIpfColorAngleLimits(etaMin);
  if(angleLimitsAtMin[2] > chiMax)
  {
    chiMax = angleLimitsAtMin[2];
  }

  std::array<double, 4> sstBBox = {etaMin, etaMax, 0.0, chiMax};
```

Then update the three `computeIPFIntensity` calls to pass `&sstBBox`:

```cpp
  ebsdlib::DoubleArrayType::Pointer intensity0 = InversePoleFigureUtilities::computeIPFIntensity(*this, dirs0.get(), imageDim, imageDim, config.lambertDim, config.normalizeMRD, &sstBBox);
  ebsdlib::DoubleArrayType::Pointer intensity1 = InversePoleFigureUtilities::computeIPFIntensity(*this, dirs1.get(), imageDim, imageDim, config.lambertDim, config.normalizeMRD, &sstBBox);
  ebsdlib::DoubleArrayType::Pointer intensity2 = InversePoleFigureUtilities::computeIPFIntensity(*this, dirs2.get(), imageDim, imageDim, config.lambertDim, config.normalizeMRD, &sstBBox);
```

- [ ] **Step 2: Build everything**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all 2>&1 | tail -10
```
Expected: Clean build.

- [ ] **Step 3: Test with the Iron BCC CTF file**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && ./Bin/generate_ipf_from_file "/Users/mjackson/Applications/NXData/Data/T12-MAI-2010/fw-ar-IF1-aptr12-corr.ctf" /tmp/ipf_sst_zoomed 2>&1
```

Convert the output to PNG and visually inspect. The SST triangle should now fill the frame, with labels aligned to the corners.

- [ ] **Step 4: Test with generate_ipf_density (random orientations)**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && ./Bin/generate_ipf_density /tmp/ipf_density_zoomed 500 2>&1
```

Check output for all 11 Laue classes — each should have a well-filled triangle.

- [ ] **Step 5: Run all unit tests**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && ctest -R "EbsdLib::" 2>&1 | tail -5
```
Expected: All 296 tests pass.

- [ ] **Step 6: Commit**

```bash
cd /Users/mjackson/Workspace1/EbsdLib
git add Source/EbsdLib/LaueOps/LaueOps.cpp
git commit -m "ENH: Pass SST bounding box to computeIPFIntensity for zoomed density images"
```

---

## Important Notes

### Negative eta ranges (TrigonalOps, TrigonalLowOps)

TrigonalOps has etaMin=-90°, etaMax=-30°. TrigonalLowOps has etaMin=-120°, etaMax=0°. The pixel mapping formula `eta = etaMin + fraction * (etaMax - etaMin)` handles negative ranges correctly since it's a simple linear interpolation. The `inUnitTriangle` check will correctly identify which pixels are inside the SST.

### Cubic dynamic chiMax

For CubicOps and CubicLowOps, the SST has a curved upper boundary where chiMax varies with eta. The bounding box uses the maximum chiMax (at eta=45° for CubicHigh). Pixels inside the bounding box but outside the curved boundary will have `inUnitTriangle` return false and render as white — identical to how the legend handles it.

### Backward compatibility

The `computeIPFIntensity()` change uses a default parameter (`nullptr`). All existing callers (including the non-annotated `generateInversePoleFigure()`) continue to work unchanged with the full-disk mapping.

### No changes to annotation positioning

The `annotateIPFImage()` method and `drawIPFAnnotations()` overrides already place Miller index labels at positions relative to the canvas. Since the density image now fills the same region as the legend, the labels should align correctly with the triangle corners.
