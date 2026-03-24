# SST-Zoomed Inverse Pole Figure Density — Design Spec

## Problem

The IPF density images map pixels to the full Lambert equal-area hemisphere disk, but the Standard Stereographic Triangle (SST) occupies only a small fraction of the full disk. For cubic symmetry, the SST is roughly 1/48th of the hemisphere. This produces a tiny triangle in a large white image. The IPF legend, by contrast, zooms to fill the frame with just the SST region.

## Solution

Add a virtual method `getSSTBoundingBox()` to LaueOps that returns the spherical coordinate bounds (etaMin, etaMax, chiMin, chiMax) of each symmetry class's SST. Modify `computeIPFIntensity()` to accept an optional bounding box parameter. When provided, pixels map to only the SST bounding box region in (eta, chi) space instead of the full Lambert disk.

The Lambert binning of crystal directions (accumulation step) is unchanged. Only the output pixel-to-sphere mapping changes.

## New Virtual Method

```cpp
virtual std::array<double, 4> getSSTBoundingBox() const;
// Returns {etaMin, etaMax, chiMin, chiMax} in radians
```

### Per-Subclass Values

| Subclass | etaMax (deg) | chiMax (deg) |
|----------|-------------|-------------|
| Cubic High (m-3m) | 45 | 54.7356 (arccos(1/sqrt(3))) |
| Cubic Low (m-3) | 45 | 54.7356 |
| Hexagonal High (6/mmm) | 30 | 90 |
| Hexagonal Low (6/m) | 30 | 90 |
| Trigonal High (-3m) | 30 | 90 |
| Trigonal Low (-3) | 60 | 90 |
| Tetragonal High (4/mmm) | 45 | 90 |
| Tetragonal Low (4/m) | 45 | 90 |
| Orthorhombic (mmm) | 90 | 90 |
| Monoclinic (2/m) | 90 | 90 |
| Triclinic (-1) | 180 | 90 |

All subclasses have etaMin = 0, chiMin = 0. For cubic classes, the SST has a curved upper chi boundary that varies with eta; the bounding box uses the maximum chiMax. The existing `inUnitTriangle()` check marks pixels outside the curved boundary as white.

## Modified computeIPFIntensity Signature

```cpp
static DoubleArrayType::Pointer computeIPFIntensity(
    const LaueOps& ops,
    FloatArrayType* ipfDirections,
    int imageWidth, int imageHeight,
    int lambertDim, bool normalizeMRD,
    const std::array<double, 4>* sstBoundingBox = nullptr);
```

When `sstBoundingBox` is nullptr: existing full-disk behavior (backward compatible).
When provided: zoomed SST behavior.

## Pixel-to-Sphere Mapping (Zoomed SST Mode)

```
For each pixel (px, py) in [0, imageWidth) x [0, imageHeight):
  eta = etaMin + ((px + 0.5) / imageWidth) * (etaMax - etaMin)
  chi = chiMin + ((py + 0.5) / imageHeight) * (chiMax - chiMin)

  xyz = (sin(chi)*cos(eta), sin(chi)*sin(eta), cos(chi))

  if !inUnitTriangle(eta, chi):
    intensity = -1.0  (white)
  else:
    intensity = lambert.getInterpolatedValue(xyz)
```

The +0.5 offset centers the sample at each pixel center.

## Changes to generateAnnotatedIPFDensity

Call `getSSTBoundingBox()` and pass it to `computeIPFIntensity()`. No other changes needed — the output image fills the frame with the SST, and the annotation labels from `drawIPFAnnotations()` align correctly.

## Files to Modify

- `Source/EbsdLib/LaueOps/LaueOps.h` — add `getSSTBoundingBox()` virtual declaration
- `Source/EbsdLib/LaueOps/LaueOps.cpp` — update `generateAnnotatedIPFDensity()` to pass bounding box; add default `getSSTBoundingBox()` implementation
- `Source/EbsdLib/Utilities/InversePoleFigureUtilities.h` — update `computeIPFIntensity()` signature
- `Source/EbsdLib/Utilities/InversePoleFigureUtilities.cpp` — implement zoomed SST mapping
- 11 LaueOps subclass `.h` and `.cpp` files — add `getSSTBoundingBox()` override
