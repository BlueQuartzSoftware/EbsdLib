# Annotated Inverse Pole Figure Density Images — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add proper labeling (title, Miller index annotations, color bar) to Inverse Pole Figure density images by refactoring the existing `generateIPFTriangleLegend` scaffolding into shared code that both the IPF legend and IPF density features can use.

**Architecture:** Extract the ~40 lines of identical canvas setup / teardown code from all 11 `generateIPFTriangleLegend()` implementations into a shared non-virtual base-class method. Promote each subclass's `DrawFullCircleAnnotations()` free function to a virtual method so the base class can call it. Both `generateIPFTriangleLegend()` and a new `generateAnnotatedIPFDensity()` method call the same shared annotation pipeline, differing only in how the triangle image is produced and whether a color bar is added.

**Tech Stack:** C++20, canvas_ity (2D rendering), EbsdLib LaueOps class hierarchy, EbsdDataArray

---

## File Map

### Files to Modify

| File | Change |
|------|--------|
| `Source/EbsdLib/LaueOps/LaueOps.h` | Add `drawIPFAnnotations()` pure virtual declaration. Add `annotateIPFImage()` protected non-virtual helper. Add `adjustFigureOrigin()` virtual method. Add `generateAnnotatedIPFDensity()` public method declaration. |
| `Source/EbsdLib/LaueOps/LaueOps.cpp` | Implement `annotateIPFImage()` (shared scaffolding). Implement `generateAnnotatedIPFDensity()` (density pipeline + annotation + color bar). |
| `Source/EbsdLib/LaueOps/CubicOps.h` | Declare `drawIPFAnnotations()` and `adjustFigureOrigin()` overrides. |
| `Source/EbsdLib/LaueOps/CubicOps.cpp` | Move `DrawFullCircleAnnotations` body into `drawIPFAnnotations()` override. Refactor `generateIPFTriangleLegend()` to call `annotateIPFImage()`. |
| `Source/EbsdLib/LaueOps/CubicLowOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/CubicLowOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/HexagonalOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/HexagonalOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/HexagonalLowOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/HexagonalLowOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/TrigonalOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/TrigonalOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/TrigonalLowOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/TrigonalLowOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/TetragonalOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/TetragonalOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/TetragonalLowOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/TetragonalLowOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/OrthoRhombicOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/OrthoRhombicOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/MonoclinicOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/MonoclinicOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/EbsdLib/LaueOps/TriclinicOps.h` | Same as CubicOps.h |
| `Source/EbsdLib/LaueOps/TriclinicOps.cpp` | Same pattern as CubicOps.cpp |
| `Source/Apps/generate_ipf_from_file.cpp` | Update to call `generateAnnotatedIPFDensity()` instead of raw `generateInversePoleFigure()`. |
| `Source/Apps/generate_ipf_density.cpp` | Update to call `generateAnnotatedIPFDensity()` instead of raw `generateInversePoleFigure()`. |

### Files to Read (reference only, no changes)

| File | Why |
|------|-----|
| `Source/EbsdLib/Utilities/CanvasUtilities.hpp` | Contains `WriteText`, `DrawLine`, `MirrorImage`, `ConvertColorOrder`, `RemoveAlphaChannel`, `CropRGBImage` |
| `Source/EbsdLib/Utilities/Fonts.hpp` | Contains `GetLatoBold()`, `GetLatoRegular()` |
| `Source/EbsdLib/Utilities/InversePoleFigureUtilities.h` | Contains `InversePoleFigureConfiguration_t`, `computeIPFDirections`, `computeIPFIntensity`, `createIPFColorImage` |
| `Source/EbsdLib/Utilities/ColorTable.h` | Color table for color bar rendering |
| `Source/EbsdLib/Utilities/TiffWriter.h` | Writing TIFF output from apps |

---

## Background: Current Architecture

### generateIPFTriangleLegend() — Current flow (duplicated 11 times)

Each of the 11 LaueOps subclasses has an identical ~90-line `generateIPFTriangleLegend()` that:

1. Computes margins, legend dimensions, figureOrigin (2-3 lines **vary per subclass**)
2. Calls `CreateIPFLegend(this, legendHeight, generateEntirePlane)` — file-scoped free function (**varies per subclass** — different SST geometry)
3. Calls `ConvertColorOrder()` + `MirrorImage()` — **identical**
4. Creates canvas, fills white background, sets up fonts — **identical** (~20 lines)
5. Draws legend image onto canvas — **identical**
6. Draws title — **identical**
7. Calls `DrawFullCircleAnnotations()` — file-scoped free function (**varies per subclass** — different Miller indices and positions)
8. Extracts RGBA, removes alpha — **identical**

Only steps 1, 2, and 7 vary. Steps 3-6 and 8 are copy-pasted across all 11 files.

### generateInversePoleFigure() — Current flow (single implementation in base class)

A non-virtual method in `LaueOps.cpp` that:
1. Computes IPF directions for 3 sample directions
2. Computes intensity via Lambert projection
3. Finds global min/max across all 3 images
4. Creates RGBA color images (colored SST, white outside)

Returns raw ARGB images — **no annotations, no title, no labels, no color bar**.

---

## Design: Refactored Architecture

### New virtual methods on LaueOps

```cpp
// In LaueOps.h:

/**
 * @brief Per-subclass hook that draws Miller index labels and SST boundary
 * annotations onto a canvas_ity canvas. Replaces the file-scoped
 * DrawFullCircleAnnotations() free functions.
 */
virtual void drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim,
    float fontPtSize, const std::vector<float>& margins,
    std::array<float, 2> figureOrigin,
    std::array<float, 2> figureCenter,
    bool drawFullCircle) const = 0;

/**
 * @brief Per-subclass hook that returns the figureOrigin adjustment
 * when rendering the SST-only view (generateEntirePlane == false).
 * Default returns the base figureOrigin unchanged.
 */
virtual std::array<float, 2> adjustFigureOrigin(
    std::array<float, 2> figureOrigin,
    int legendWidth, int legendHeight,
    const std::vector<float>& margins, float fontPtSize,
    bool generateEntirePlane) const;
```

### New shared scaffolding method (non-virtual, protected)

```cpp
/**
 * @brief Shared canvas scaffolding used by both generateIPFTriangleLegend()
 * and generateAnnotatedIPFDensity(). Takes a pre-rendered triangle image
 * (ARGB, square), annotates it with title + per-subclass Miller index labels,
 * and returns the final RGB image.
 */
UInt8ArrayType::Pointer annotateIPFImage(
    UInt8ArrayType::Pointer triangleImage,
    int imageDim,
    int canvasDim,
    const std::string& title,
    bool generateEntirePlane) const;
```

### New public method for annotated density

```cpp
/**
 * @brief Generates 3 annotated inverse pole figure density images with
 * title, Miller index labels, and MRD color bar.
 */
std::vector<UInt8ArrayType::Pointer> generateAnnotatedIPFDensity(
    InversePoleFigureConfiguration_t& config) const;
```

### Data flow after refactor

**IPF Legend:**
```
CreateIPFLegend()           [per-subclass, existing]
    → annotateIPFImage()    [shared scaffolding, NEW]
        → drawIPFAnnotations()  [per-subclass virtual, promoted from free function]
    → return annotated image
```

**IPF Density:**
```
generateInversePoleFigure()        [existing, produces raw ARGB images]
    → annotateIPFImage()           [shared scaffolding, same as legend]
        → drawIPFAnnotations()     [per-subclass virtual, same as legend]
        → drawColorBar()           [shared, NEW, density-specific]
    → return annotated images
```

---

## Tasks

### Task 1: Add new virtual methods to LaueOps.h

**Files:**
- Modify: `Source/EbsdLib/LaueOps/LaueOps.h:328` (near existing `generateIPFTriangleLegend` declaration)

- [ ] **Step 1: Add the `#include` for canvas_ity in LaueOps.h**

Add near the top of LaueOps.h with other includes:
```cpp
#include <canvas_ity.hpp>
```

Note: canvas_ity.hpp is already a public dependency of EbsdLib (included in CanvasUtilities.hpp, installed to include/EbsdLib). Check that it's not already included; if not, add it.

- [ ] **Step 2: Add the three new method declarations**

After the existing `generateIPFTriangleLegend` declaration (line 328), add:

```cpp
  /**
   * @brief Per-subclass hook that draws Miller index labels and SST boundary
   * annotations onto a canvas. Called by annotateIPFImage().
   */
  virtual void drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim,
      float fontPtSize, const std::vector<float>& margins,
      std::array<float, 2> figureOrigin,
      std::array<float, 2> figureCenter,
      bool drawFullCircle) const = 0;

  /**
   * @brief Per-subclass hook that adjusts the figureOrigin when rendering
   * SST-only view. Each subclass overrides to position its triangle shape
   * correctly within the canvas. Default returns figureOrigin unchanged.
   */
  virtual std::array<float, 2> adjustFigureOrigin(
      std::array<float, 2> figureOrigin,
      int legendWidth, int legendHeight,
      const std::vector<float>& margins, float fontPtSize,
      bool generateEntirePlane) const;

  /**
   * @brief Generates 3 annotated inverse pole figure density images with
   * title, Miller index labels, and MRD color bar.
   * @param config Configuration struct; imageWidth must equal imageHeight (square images required)
   * @param outMinMax Optional output for the global [min, max] intensity values
   */
  std::vector<UInt8ArrayType::Pointer> generateAnnotatedIPFDensity(
      InversePoleFigureConfiguration_t& config,
      std::pair<double, double>* outMinMax = nullptr) const;

protected:
  /**
   * @brief Shared annotation scaffolding. Takes a pre-rendered ARGB triangle
   * image, creates a canvas with white background, draws the image, adds
   * title and per-subclass annotations, returns final RGB image.
   * @param triangleImage Pre-rendered ARGB image (square, imageDim x imageDim)
   * @param imageDim Pixel dimension of the triangle image (square)
   * @param canvasDim Pixel dimension of the output canvas (square)
   * @param title Text to draw as the title
   * @param generateEntirePlane true = full circle view, false = SST only
   * @return RGB image (canvasDim x canvasDim, 3 components)
   */
  UInt8ArrayType::Pointer annotateIPFImage(
      UInt8ArrayType::Pointer triangleImage,
      int imageDim,
      int canvasDim,
      const std::string& title,
      bool generateEntirePlane) const;
```

Note: The `protected:` label is needed so subclasses can call `annotateIPFImage()`. Check the existing access specifiers in LaueOps.h and place appropriately. The existing class may not have a `protected:` section — if so, add one before the new method. The `public:` methods (`drawIPFAnnotations`, `adjustFigureOrigin`, `generateAnnotatedIPFDensity`) go in the existing `public:` section.

- [ ] **Step 3: Build to verify the header compiles**

Run:
```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target EbsdLib 2>&1 | tail -5
```
Expected: Linker errors about undefined references to the new methods (that's fine — implementations come in later tasks). If there are compiler errors, fix them first.

- [ ] **Step 4: Commit**

```bash
git add Source/EbsdLib/LaueOps/LaueOps.h
git commit -m "ENH: Add virtual method declarations for shared IPF annotation pipeline"
```

---

### Task 2: Implement `annotateIPFImage()` and `adjustFigureOrigin()` in LaueOps.cpp

**Files:**
- Modify: `Source/EbsdLib/LaueOps/LaueOps.cpp` (after existing `generateInversePoleFigure`)

- [ ] **Step 1: Add includes to LaueOps.cpp**

Add these includes at the top of LaueOps.cpp if not already present:
```cpp
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/Fonts.hpp"
#include <canvas_ity.hpp>
```

- [ ] **Step 2: Implement the default `adjustFigureOrigin()`**

Add after `generateInversePoleFigure()`:
```cpp
std::array<float, 2> LaueOps::adjustFigureOrigin(
    std::array<float, 2> figureOrigin,
    int legendWidth, int legendHeight,
    const std::vector<float>& margins, float fontPtSize,
    bool generateEntirePlane) const
{
  return figureOrigin;
}
```

This default implementation returns the origin unchanged. Subclasses with SST positioning needs will override it.

- [ ] **Step 3: Implement `annotateIPFImage()`**

This is the shared scaffolding extracted from the 11 copies of `generateIPFTriangleLegend()`. Add after `adjustFigureOrigin()`:

```cpp
UInt8ArrayType::Pointer LaueOps::annotateIPFImage(
    UInt8ArrayType::Pointer triangleImage,
    int imageDim,
    int canvasDim,
    const std::string& title,
    bool generateEntirePlane) const
{
  // Compute layout
  const float fontPtSize = static_cast<float>(canvasDim) / 24.0f;
  const std::vector<float> margins = {
      fontPtSize * 3,                        // Top
      static_cast<float>(canvasDim / 7.0f),  // Right
      fontPtSize * 2,                        // Bottom
      static_cast<float>(canvasDim / 7.0f)   // Left
  };

  int legendHeight = canvasDim - static_cast<int>(margins[0]) - static_cast<int>(margins[2]);
  int legendWidth = canvasDim - static_cast<int>(margins[1]) - static_cast<int>(margins[3]);

  if(legendHeight > legendWidth)
  {
    legendHeight = legendWidth;
  }
  else
  {
    legendWidth = legendHeight;
  }

  int halfWidth = legendWidth / 2;
  int halfHeight = legendHeight / 2;

  // Compute figure origin — subclass may override for SST positioning
  std::array<float, 2> figureOrigin = {margins[3], margins[0] * 1.33F};
  figureOrigin = adjustFigureOrigin(figureOrigin, legendWidth, legendHeight, margins, fontPtSize, generateEntirePlane);

  std::array<float, 2> figureCenter = {figureOrigin[0] + halfWidth, figureOrigin[1] + halfHeight};

  // Scale the triangle image to legend dimensions if needed
  // The input image is imageDim x imageDim; we need legendHeight x legendHeight
  // For now we assume the caller provides an image at the correct size.
  // Convert from ARGB to RGBA for canvas_ity
  ebsdlib::UInt8ArrayType::Pointer image = ebsdlib::ConvertColorOrder(triangleImage.get(), imageDim);
  // Mirror across X axis (image was drawn with +Y pointing down)
  image = ebsdlib::MirrorImage(image.get(), imageDim);

  // Create canvas
  canvas_ity::canvas context(canvasDim, canvasDim);

  std::vector<unsigned char> latoBold = ebsdlib::fonts::GetLatoBold();
  std::vector<unsigned char> latoRegular = ebsdlib::fonts::GetLatoRegular();
  context.set_font(latoBold.data(), static_cast<int>(latoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;

  // Fill background with white
  context.move_to(0.0f, 0.0f);
  context.line_to(static_cast<float>(canvasDim), 0.0f);
  context.line_to(static_cast<float>(canvasDim), static_cast<float>(canvasDim));
  context.line_to(0.0f, static_cast<float>(canvasDim));
  context.line_to(0.0f, 0.0f);
  context.close_path();
  context.set_color(canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f);
  context.fill();

  // Draw the triangle image onto the canvas
  context.draw_image(image->getPointer(0), imageDim, imageDim,
                     imageDim * image->getNumberOfComponents(),
                     figureOrigin[0], figureOrigin[1],
                     static_cast<float>(legendWidth),
                     static_cast<float>(legendHeight));

  // Draw title
  context.set_font(latoBold.data(), static_cast<int>(latoBold.size()), fontPtSize * 1.5);
  ebsdlib::WriteText(context, title, {margins[0], static_cast<float>(fontPtSize * 1.5)}, fontPtSize * 1.5);

  // Draw per-subclass annotations (Miller indices, SST boundary lines)
  context.set_font(latoRegular.data(), static_cast<int>(latoRegular.size()), fontPtSize);
  drawIPFAnnotations(context, canvasDim, fontPtSize, margins, figureOrigin, figureCenter, generateEntirePlane);

  // Extract rendered pixels and remove alpha channel
  ebsdlib::UInt8ArrayType::Pointer rgbaCanvasImage = ebsdlib::UInt8ArrayType::CreateArray(
      canvasDim * canvasDim, {4ULL}, "Annotated IPF", true);
  context.get_image_data(rgbaCanvasImage->getPointer(0), canvasDim, canvasDim, canvasDim * 4, 0, 0);

  return ebsdlib::RemoveAlphaChannel(rgbaCanvasImage.get());
}
```

- [ ] **Step 4: Build to check for compilation errors**

Run:
```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target EbsdLib 2>&1 | tail -20
```
Expected: Linker errors for the pure virtual `drawIPFAnnotations` in the subclasses (they don't implement it yet). That's expected.

- [ ] **Step 5: Commit**

```bash
git add Source/EbsdLib/LaueOps/LaueOps.cpp
git commit -m "ENH: Implement shared annotateIPFImage() scaffolding in LaueOps base class"
```

---

### Task 3: Refactor CubicOps — promote DrawFullCircleAnnotations to virtual override

This task establishes the pattern for all 11 subclasses. Do CubicOps first, verify it works, then apply the same pattern to the remaining 10.

**Files:**
- Modify: `Source/EbsdLib/LaueOps/CubicOps.h`
- Modify: `Source/EbsdLib/LaueOps/CubicOps.cpp`

- [ ] **Step 1: Add virtual method declarations to CubicOps.h**

Add near the existing `generateIPFTriangleLegend` declaration:
```cpp
  void drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim,
      float fontPtSize, std::vector<float> margins,
      std::array<float, 2> figureOrigin,
      std::array<float, 2> figureCenter,
      bool drawFullCircle) const override;

  std::array<float, 2> adjustFigureOrigin(
      std::array<float, 2> figureOrigin,
      int legendWidth, int legendHeight,
      const std::vector<float>& margins, float fontPtSize,
      bool generateEntirePlane) const override;
```

Also add `#include <canvas_ity.hpp>` if not already present. Check the existing includes — CubicOps.cpp includes it but CubicOps.h may not.

- [ ] **Step 2: Implement `adjustFigureOrigin()` override in CubicOps.cpp**

CubicOps adjusts only `figureOrigin[1]` when `generateEntirePlane == false`:

```cpp
std::array<float, 2> CubicOps::adjustFigureOrigin(
    std::array<float, 2> figureOrigin,
    int legendWidth, int legendHeight,
    const std::vector<float>& margins, float fontPtSize,
    bool generateEntirePlane) const
{
  if(!generateEntirePlane)
  {
    figureOrigin[1] = 0.0F + fontPtSize * 2.0F;
  }
  return figureOrigin;
}
```

- [ ] **Step 3: Convert DrawFullCircleAnnotations to `drawIPFAnnotations()` override**

Rename the existing file-scoped `DrawFullCircleAnnotations()` function in CubicOps.cpp to the virtual override `CubicOps::drawIPFAnnotations()`. The function body stays identical — only the function signature changes:

Before:
```cpp
void DrawFullCircleAnnotations(canvas_ity::canvas& context, int canvasDim, float fontPtSize, std::vector<float> margins, std::array<float, 2> figureOrigin, std::array<float, 2> figureCenter,
                               bool drawFullCircle)
```

After:
```cpp
void CubicOps::drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim, float fontPtSize, std::vector<float> margins, std::array<float, 2> figureOrigin, std::array<float, 2> figureCenter,
                                  bool drawFullCircle) const
```

**Important:** CubicOps has special handling — when `drawFullCircle == false`, it adjusts `figureCenter` before drawing labels (see lines 2153 in current code). This logic is already inside `DrawFullCircleAnnotations` itself in CubicOps. Verify by reading the function body that the figureCenter adjustment is handled internally. If the adjustment is done OUTSIDE the function (in `generateIPFTriangleLegend` before calling it), then move that logic INTO the new `drawIPFAnnotations` override:

```cpp
// If CubicOps did this in generateIPFTriangleLegend:
//   figureCenter = {figureOrigin[0], figureOrigin[1] + legendHeight};
// Then add it at the top of drawIPFAnnotations:
if(!drawFullCircle)
{
  figureCenter = {figureOrigin[0], figureOrigin[1] + static_cast<float>(/* legendHeight */)};
}
```

Note: The `legendHeight` value isn't directly available in `drawIPFAnnotations`. However, looking at the existing code, `figureCenter` is computed from `figureOrigin + halfWidth/halfHeight`, which means the caller (annotateIPFImage) already computes it. For CubicOps, when `!drawFullCircle`, it overrides figureCenter to `{figureOrigin[0], figureOrigin[1] + legendHeight}`. We can compute this from the available parameters: `legendHeight = canvasDim - margins[0] - margins[2]` (clamped to square). Add this computation at the top of the override if needed.

- [ ] **Step 4: Refactor `generateIPFTriangleLegend()` to use `annotateIPFImage()`**

Replace the body of `CubicOps::generateIPFTriangleLegend()` with:

```cpp
ebsdlib::UInt8ArrayType::Pointer CubicOps::generateIPFTriangleLegend(int canvasDim, bool generateEntirePlane) const
{
  // Compute legend dimensions (same formula as annotateIPFImage uses)
  const float fontPtSize = static_cast<float>(canvasDim) / 24.0f;
  int legendHeight = canvasDim - static_cast<int>(fontPtSize * 3) - static_cast<int>(fontPtSize * 2);
  int legendWidth = canvasDim - static_cast<int>(canvasDim / 7.0f) * 2;
  if(legendHeight > legendWidth)
  {
    legendHeight = legendWidth;
  }
  else
  {
    legendWidth = legendHeight;
  }

  // Generate the colored SST triangle image (ARGB)
  ebsdlib::UInt8ArrayType::Pointer image = CreateIPFLegend(this, legendHeight, generateEntirePlane);

  // Annotate with title and Miller index labels
  return annotateIPFImage(image, legendHeight, canvasDim, getSymmetryName(), generateEntirePlane);
}
```

- [ ] **Step 5: Build and run the generate_ipf_legends app to verify output matches**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target generate_ipf_legends 2>&1 | tail -5
```

This will fail to link because the other 10 subclasses don't implement `drawIPFAnnotations` yet. That's expected. To verify CubicOps in isolation, we need to complete all 11 subclasses first (Task 4).

- [ ] **Step 6: Commit**

```bash
git add Source/EbsdLib/LaueOps/CubicOps.h Source/EbsdLib/LaueOps/CubicOps.cpp
git commit -m "ENH: Refactor CubicOps to use shared annotation pipeline"
```

---

### Task 4: Refactor remaining 10 LaueOps subclasses

Apply the same pattern from Task 3 to each remaining subclass. Each subclass needs:

1. Add `drawIPFAnnotations()` and `adjustFigureOrigin()` override declarations to the header
2. Convert the file-scoped `DrawFullCircleAnnotations()` to the `drawIPFAnnotations()` virtual override (same body, new signature with `const` qualifier and class prefix)
3. Implement `adjustFigureOrigin()` with the subclass-specific figureOrigin adjustment
4. Refactor `generateIPFTriangleLegend()` to call `annotateIPFImage()`

**Per-subclass figureOrigin adjustments** (from the existing code):

| Subclass | adjustFigureOrigin when !generateEntirePlane |
|----------|---------------------------------------------|
| CubicOps | `figureOrigin[1] = fontPtSize * 2.0F` |
| CubicLowOps | `figureOrigin[1] = fontPtSize * 2.0F` |
| HexagonalOps | `figureOrigin[0] = -margins[3] * 0.5F; figureOrigin[1] = -halfHeight + margins[0] + fontPtSize` |
| HexagonalLowOps | `figureOrigin[0] = -halfWidth * 0.25F; figureOrigin[1] = margins[0]` |
| TrigonalOps | `figureOrigin[0] = -halfWidth * 0.25; figureOrigin[1] = -halfHeight * 0.5` |
| TrigonalLowOps | `figureOrigin[0] = -legendWidth * 0.0F; figureOrigin[1] = -legendHeight * 0.25F` |
| TetragonalOps | `figureOrigin[0] = -margins[2]; figureOrigin[1] = fontPtSize * 2.0F` |
| TetragonalLowOps | `figureOrigin[0] = -margins[3]` (Y unchanged) |
| OrthoRhombicOps | `figureOrigin[0] = -margins[3]` (Y unchanged) |
| MonoclinicOps | No adjustment (use default) |
| TriclinicOps | No adjustment (use default) |

Note: MonoclinicOps and TriclinicOps have commented-out adjustments in the existing code. They use the default figureOrigin, so they do not need to override `adjustFigureOrigin()`.

**Files (for each subclass):**
- Modify: `Source/EbsdLib/LaueOps/<SubclassName>.h`
- Modify: `Source/EbsdLib/LaueOps/<SubclassName>.cpp`

- [ ] **Step 1: Refactor CubicLowOps**

Follow Task 3 pattern. CubicLowOps has the same figureOrigin adjustment as CubicOps AND the same special figureCenter handling (if/else on generateEntirePlane before calling DrawFullCircleAnnotations). Make sure to handle the figureCenter adjustment inside `drawIPFAnnotations()`.

- [ ] **Step 2: Refactor HexagonalOps**

Follow Task 3 pattern. HexagonalOps has unique figureOrigin adjustment (both X and Y). No special figureCenter handling.

- [ ] **Step 3: Refactor HexagonalLowOps**

Follow Task 3 pattern.

- [ ] **Step 4: Refactor TrigonalOps**

Follow Task 3 pattern.

- [ ] **Step 5: Refactor TrigonalLowOps**

Follow Task 3 pattern.

- [ ] **Step 6: Refactor TetragonalOps**

Follow Task 3 pattern.

- [ ] **Step 7: Refactor TetragonalLowOps**

Follow Task 3 pattern.

- [ ] **Step 8: Refactor OrthoRhombicOps**

Follow Task 3 pattern. OrthoRhombicOps adjusts `figureOrigin[0] = -margins[3]`.

- [ ] **Step 9: Refactor MonoclinicOps**

Follow Task 3 pattern. No `adjustFigureOrigin` override needed (uses default). Still need `drawIPFAnnotations` override.

- [ ] **Step 10: Refactor TriclinicOps**

Follow Task 3 pattern. No `adjustFigureOrigin` override needed (uses default). Still need `drawIPFAnnotations` override.

- [ ] **Step 11: Build the full library**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target EbsdLib 2>&1 | tail -10
```
Expected: Clean build with no errors.

- [ ] **Step 12: Build and run generate_ipf_legends to verify legend output is unchanged**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target generate_ipf_legends && ./Bin/generate_ipf_legends 2>&1
```

Visually compare the output images in `Testing/Temporary/IPF_Legend/` against the reference images at:
```
/Users/mjackson/Workspace1/DREAM3D-Build/NX-Com-Qt69-Vtk95-Dbg/simplnx/EbsdLib/Testing/Temporary/IPF_Legend/
```

Each Laue class directory should contain a `<ClassName>.tiff` and `<ClassName>_FULL.tiff` that match the reference visually. Pay special attention to:
- Label positions (Miller indices at correct corners)
- Triangle orientation and cropping
- Title text

- [ ] **Step 13: Commit**

```bash
git add Source/EbsdLib/LaueOps/*.h Source/EbsdLib/LaueOps/*.cpp
git commit -m "ENH: Refactor all 11 LaueOps subclasses to use shared annotation pipeline"
```

---

### Task 5: Implement `generateAnnotatedIPFDensity()` with color bar

**Files:**
- Modify: `Source/EbsdLib/LaueOps/LaueOps.cpp`

- [ ] **Step 1: Implement `generateAnnotatedIPFDensity()`**

This method inlines the key parts of `generateInversePoleFigure()` to avoid double-computing the expensive intensity step. It computes directions + intensity, extracts global min/max for the color bar, creates the color images, then annotates.

**Important:** `config.imageWidth` must equal `config.imageHeight` (square images required) because `ConvertColorOrder` and `MirrorImage` assume square dimensions.

The `canvasDim` is computed from `imageDim` so that `legendWidth == imageDim` (no lossy scaling):
```
legendWidth = canvasDim - 2 * (canvasDim / 7)
```
Solving for `canvasDim` when `legendWidth == imageDim`: `canvasDim = imageDim * 7 / 5`

Add after `annotateIPFImage()` in LaueOps.cpp:

```cpp
std::vector<UInt8ArrayType::Pointer> LaueOps::generateAnnotatedIPFDensity(
    InversePoleFigureConfiguration_t& config,
    std::pair<double, double>* outMinMax) const
{
  // Require square images (ConvertColorOrder and MirrorImage assume square)
  if(config.imageWidth != config.imageHeight)
  {
    std::cerr << "generateAnnotatedIPFDensity: imageWidth must equal imageHeight" << std::endl;
    return {};
  }
  int imageDim = config.imageWidth;

  // Step 1: Compute IPF directions and intensity for all 3 sample directions
  std::array<ebsdlib::FloatArrayType::Pointer, 3> dirs;
  std::array<ebsdlib::DoubleArrayType::Pointer, 3> intensities;
  for(size_t i = 0; i < 3; i++)
  {
    dirs[i] = InversePoleFigureUtilities::computeIPFDirections(*this, config.eulers, config.sampleDirections[i]);
    intensities[i] = InversePoleFigureUtilities::computeIPFIntensity(*this, dirs[i].get(), config.imageWidth, config.imageHeight, config.lambertDim, config.normalizeMRD);
  }

  // Step 2: Find global min/max across all 3 intensity images
  double globalMin = std::numeric_limits<double>::max();
  double globalMax = std::numeric_limits<double>::lowest();
  for(auto& intensity : intensities)
  {
    double* dPtr = intensity->getPointer(0);
    size_t count = intensity->getNumberOfTuples();
    for(size_t i = 0; i < count; ++i)
    {
      if(dPtr[i] >= 0.0)
      {
        globalMin = std::min(globalMin, dPtr[i]);
        globalMax = std::max(globalMax, dPtr[i]);
      }
    }
  }
  if(globalMax < globalMin)
  {
    globalMin = 0.0;
    globalMax = 1.0;
  }
  if(outMinMax != nullptr)
  {
    *outMinMax = {globalMin, globalMax};
  }

  // Step 3: Create ARGB color images and annotate each one
  // Compute canvasDim so legendWidth == imageDim (no scaling):
  //   legendWidth = canvasDim - 2 * floor(canvasDim / 7)
  //   We want legendWidth == imageDim, so canvasDim ~= imageDim * 7 / 5
  int canvasDim = static_cast<int>(std::ceil(static_cast<double>(imageDim) * 7.0 / 5.0));

  std::vector<UInt8ArrayType::Pointer> annotatedImages(3);
  std::array<std::string, 3> defaultLabels = {"IPF-0", "IPF-1", "IPF-2"};

  for(size_t i = 0; i < 3; i++)
  {
    std::string label = (i < config.labels.size()) ? config.labels[i] : defaultLabels[i];
    std::string title = config.phaseName + " - " + label;

    // Create ARGB color image
    std::vector<size_t> dims = {4};
    ebsdlib::UInt8ArrayType::Pointer rawImage = ebsdlib::UInt8ArrayType::CreateArray(
        static_cast<size_t>(imageDim * imageDim), dims, label, true);
    InversePoleFigureUtilities::createIPFColorImage(
        intensities[i].get(), imageDim, imageDim, config.numColors, globalMin, globalMax, rawImage.get());

    // Annotate with title and Miller index labels (SST-only view for density)
    ebsdlib::UInt8ArrayType::Pointer annotated = annotateIPFImage(
        rawImage, imageDim, canvasDim, title, false);

    // Add color bar
    annotated = drawColorBar(annotated, canvasDim, config.numColors, globalMin, globalMax, config.normalizeMRD);

    annotatedImages[i] = annotated;
  }

  return annotatedImages;
}
```

- [ ] **Step 2: Implement `drawColorBar()` helper**

Add as a private method of LaueOps (declare in LaueOps.h in the private/protected section):

```cpp
// In LaueOps.h, protected section:
  UInt8ArrayType::Pointer drawColorBar(
      UInt8ArrayType::Pointer image,
      int canvasDim,
      int numColors,
      double minValue, double maxValue,
      bool isMRD) const;
```

Implementation in LaueOps.cpp:

```cpp
UInt8ArrayType::Pointer LaueOps::drawColorBar(
    UInt8ArrayType::Pointer image,
    int canvasDim,
    int numColors,
    double minValue, double maxValue,
    bool isMRD) const
{
  const float fontPtSize = static_cast<float>(canvasDim) / 24.0f;

  // Create canvas and draw the existing image onto it
  canvas_ity::canvas context(canvasDim, canvasDim);

  std::vector<unsigned char> latoBold = ebsdlib::fonts::GetLatoBold();
  std::vector<unsigned char> latoRegular = ebsdlib::fonts::GetLatoRegular();

  // Draw the input image (RGB, 3 components) onto the canvas
  // canvas_ity expects RGBA, so we need to add alpha channel back
  size_t numPixels = image->getNumberOfTuples();
  ebsdlib::UInt8ArrayType::Pointer rgbaImage = ebsdlib::UInt8ArrayType::CreateArray(numPixels, {4ULL}, "RGBA", true);
  for(size_t i = 0; i < numPixels; i++)
  {
    uint8_t* src = image->getTuplePointer(i);
    uint8_t* dst = rgbaImage->getTuplePointer(i);
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = 255;
  }

  context.draw_image(rgbaImage->getPointer(0), canvasDim, canvasDim,
                     canvasDim * 4, 0.0f, 0.0f,
                     static_cast<float>(canvasDim), static_cast<float>(canvasDim));

  // Color bar layout
  float barX = static_cast<float>(canvasDim) - fontPtSize * 3.0f;
  float barY = fontPtSize * 4.0f;
  float barWidth = fontPtSize * 1.0f;
  float barHeight = static_cast<float>(canvasDim) - fontPtSize * 8.0f;

  // Get color table
  std::vector<float> colors;
  EbsdColorTable::GetColorTable(numColors, colors);

  // Draw color bar segments (bottom = min, top = max)
  float segmentHeight = barHeight / static_cast<float>(numColors);
  for(int c = 0; c < numColors; c++)
  {
    float y = barY + barHeight - (c + 1) * segmentHeight;
    int ci = c * 3;
    context.set_color(canvas_ity::fill_style, colors[ci], colors[ci + 1], colors[ci + 2], 1.0f);
    context.move_to(barX, y);
    context.line_to(barX + barWidth, y);
    context.line_to(barX + barWidth, y + segmentHeight);
    context.line_to(barX, y + segmentHeight);
    context.close_path();
    context.fill();
  }

  // Draw color bar outline
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.set_line_width(1.0f);
  context.move_to(barX, barY);
  context.line_to(barX + barWidth, barY);
  context.line_to(barX + barWidth, barY + barHeight);
  context.line_to(barX, barY + barHeight);
  context.close_path();
  context.stroke();

  // Draw min/max labels
  context.set_font(latoRegular.data(), static_cast<int>(latoRegular.size()), fontPtSize * 0.8f);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);

  std::ostringstream maxStr;
  maxStr << std::fixed << std::setprecision(1) << maxValue;
  context.fill_text(maxStr.str().c_str(), barX - fontPtSize * 0.5f, barY - fontPtSize * 0.3f);

  std::ostringstream minStr;
  minStr << std::fixed << std::setprecision(1) << minValue;
  context.fill_text(minStr.str().c_str(), barX - fontPtSize * 0.5f, barY + barHeight + fontPtSize);

  // Draw "MRD" or "Counts" label
  std::string unitLabel = isMRD ? "MRD" : "Counts";
  context.set_font(latoBold.data(), static_cast<int>(latoBold.size()), fontPtSize * 0.7f);
  context.fill_text(unitLabel.c_str(), barX - fontPtSize * 0.2f, barY + barHeight + fontPtSize * 2.0f);

  // Extract and return
  ebsdlib::UInt8ArrayType::Pointer result = ebsdlib::UInt8ArrayType::CreateArray(canvasDim * canvasDim, {4ULL}, "Annotated IPF Density", true);
  context.get_image_data(result->getPointer(0), canvasDim, canvasDim, canvasDim * 4, 0, 0);

  return ebsdlib::RemoveAlphaChannel(result.get());
}
```

Note: `EbsdColorTable::GetColorTable` returns float values in [0, 1] range. The `colors` vector has `numColors * 3` elements (RGB triplets). Verify this by reading `Source/EbsdLib/Utilities/ColorTable.h`.

- [ ] **Step 3: Add required include**

Add to LaueOps.cpp:
```cpp
#include <iomanip>
#include <sstream>
```

- [ ] **Step 4: Build**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target EbsdLib 2>&1 | tail -10
```
Expected: Clean build.

- [ ] **Step 5: Commit**

```bash
git add Source/EbsdLib/LaueOps/LaueOps.h Source/EbsdLib/LaueOps/LaueOps.cpp
git commit -m "ENH: Implement generateAnnotatedIPFDensity() with color bar rendering"
```

---

### Task 6: Update generate_ipf_from_file.cpp to use annotated output

**Files:**
- Modify: `Source/Apps/generate_ipf_from_file.cpp`

- [ ] **Step 1: Update `generateIPFForPhase()` to use `generateAnnotatedIPFDensity()`**

In `generate_ipf_from_file.cpp`, replace the `generateIPFForPhase()` function. The key change is calling `ops.generateAnnotatedIPFDensity(config)` instead of `ops.generateInversePoleFigure(config)`, and the returned images are now RGB (3 components) instead of ARGB (4 components), so skip the ARGB→RGB conversion:

```cpp
void generateIPFForPhase(const LaueOps& ops, ebsdlib::FloatArrayType* eulers,
    const std::string& outputDir, int imageWidth, int imageHeight,
    int lambertDim, const std::string& phaseLabel)
{
  std::string className = ops.getSymmetryName();
  std::cout << "Generating annotated IPF density for phase: " << phaseLabel
            << " (" << className << ", " << eulers->getNumberOfTuples()
            << " orientations)" << std::endl;

  InversePoleFigureConfiguration_t config;
  config.eulers = eulers;
  config.sampleDirections = {Matrix3X1D(1.0, 0.0, 0.0), Matrix3X1D(0.0, 1.0, 0.0), Matrix3X1D(0.0, 0.0, 1.0)};
  config.imageWidth = imageWidth;
  config.imageHeight = imageHeight;
  config.lambertDim = lambertDim;
  config.numColors = 64;
  config.colorMap = "Default";
  config.normalizeMRD = true;
  config.labels = {"RD", "TD", "ND"};
  config.phaseName = phaseLabel;
  config.FlipFinalImage = false;

  auto images = ops.generateAnnotatedIPFDensity(config);

  // Sanitize phase name for use as a filename
  std::string safeName = phaseLabel;
  for(auto& c : safeName)
  {
    if(c == '/' || c == '\\' || c == ' ' || c == '(' || c == ')')
    {
      c = '_';
    }
  }

  // Images are already RGB (3 components) — write directly
  // canvasDim matches the formula in generateAnnotatedIPFDensity: imageDim * 7 / 5
  std::array<std::string, 3> dirLabels = {"RD", "TD", "ND"};
  int canvasDim = static_cast<int>(std::ceil(static_cast<double>(imageWidth) * 7.0 / 5.0));
  for(size_t i = 0; i < 3; i++)
  {
    std::ostringstream filePath;
    filePath << outputDir << "/" << safeName << "_IPF_" << dirLabels[i] << ".tiff";
    auto result = TiffWriter::WriteColorImage(filePath.str(), canvasDim, canvasDim, 3, images[i]->data());
    if(result.first < 0)
    {
      std::cerr << "  ERROR writing " << filePath.str() << ": " << result.second << std::endl;
    }
    else
    {
      std::cout << "  Wrote: " << filePath.str() << std::endl;
    }
  }
}
```

Also remove the `convertARGBtoRGB()` and `writeIPFImage()` helper functions since they're no longer needed.

- [ ] **Step 2: Build and test**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target generate_ipf_from_file && ./Bin/generate_ipf_from_file "/Users/mjackson/Applications/NXData/Data/T12-MAI-2010/fw-ar-IF1-aptr12-corr.ctf" /tmp/ipf_annotated_test 2>&1
```

Visually inspect the output images at `/tmp/ipf_annotated_test/` — they should now have:
- Title at top (phase name + direction label)
- Miller index labels at SST corners
- Color bar on the right with MRD min/max values

- [ ] **Step 3: Commit**

```bash
git add Source/Apps/generate_ipf_from_file.cpp
git commit -m "ENH: Update generate_ipf_from_file to use annotated IPF density output"
```

---

### Task 7: Update generate_ipf_density.cpp to use annotated output

**Files:**
- Modify: `Source/Apps/generate_ipf_density.cpp`

- [ ] **Step 1: Update the app to use `generateAnnotatedIPFDensity()`**

Update the `generateIPFForLaueClass()` function in the same way as Task 6 — call `ops.generateAnnotatedIPFDensity(config)` and write the returned RGB images directly. Also update `generateSingleIPFForLaueClass()` similarly if desired, or leave it using the raw pipeline for comparison.

- [ ] **Step 2: Build and test**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target generate_ipf_density && ./Bin/generate_ipf_density /tmp/ipf_density_annotated 500 2>&1
```

Visually inspect the output. All 11 Laue classes should produce properly annotated images.

- [ ] **Step 3: Commit**

```bash
git add Source/Apps/generate_ipf_density.cpp
git commit -m "ENH: Update generate_ipf_density to use annotated IPF density output"
```

---

### Task 8: Run all unit tests and verify no regressions

**Files:**
- Read: `Source/Test/InversePoleFigureTest.cpp` (to understand what's tested)

- [ ] **Step 1: Build all targets**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && cmake --build . --target all 2>&1 | tail -10
```
Expected: Clean build.

- [ ] **Step 2: Run all EbsdLib tests**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && ctest -R "EbsdLib::" --verbose 2>&1
```
Expected: All tests pass.

- [ ] **Step 3: Run generate_ipf_legends and visually verify**

```bash
cd /Users/mjackson/Workspace1/DREAM3D-Build/EbsdLib-Release && ./Bin/generate_ipf_legends 2>&1
```

Compare output images with reference images to confirm the refactor didn't change the legend output.

- [ ] **Step 4: Commit any test fixes if needed**

---

## Important Notes

### No CMake changes required

All new code is added to existing source files (`LaueOps.h`, `LaueOps.cpp`, and the 11 subclass `.h`/`.cpp` files). No new source files are created. The `canvas_ity.hpp` include added to `LaueOps.h` is already a linked dependency of the EbsdLib target (via `PRIVATE` include in `SourceList.cmake`). No CMake modifications are needed.

### The figureCenter special case in CubicOps and CubicLowOps

These two subclasses adjust `figureCenter` in `generateIPFTriangleLegend()` before calling `DrawFullCircleAnnotations()` when `generateEntirePlane == false`:
```cpp
figureCenter = {figureOrigin[0], figureOrigin[1] + legendHeight};
```
This adjustment must be moved INTO the `drawIPFAnnotations()` override for these two classes, since `annotateIPFImage()` always computes `figureCenter = {figureOrigin[0] + halfWidth, figureOrigin[1] + halfHeight}`.

Recompute `legendHeight` inside `drawIPFAnnotations`:
```cpp
void CubicOps::drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim,
    float fontPtSize, const std::vector<float>& margins,
    std::array<float, 2> figureOrigin,
    std::array<float, 2> figureCenter,
    bool drawFullCircle) const
{
  if(!drawFullCircle)
  {
    // Recompute legendHeight from canvasDim and margins (same formula as annotateIPFImage)
    int legendHeight = canvasDim - static_cast<int>(margins[0]) - static_cast<int>(margins[2]);
    int legendWidth = canvasDim - static_cast<int>(margins[1]) - static_cast<int>(margins[3]);
    if(legendHeight > legendWidth) { legendHeight = legendWidth; }
    figureCenter = {figureOrigin[0], figureOrigin[1] + static_cast<float>(legendHeight)};
  }
  // ... rest of existing DrawFullCircleAnnotations body ...
}
```

### Square image requirement

`ConvertColorOrder()` and `MirrorImage()` in `CanvasUtilities.hpp` both take a single `imageDim` parameter and iterate `imageDim × imageDim` pixels. This means all input images must be square. `generateAnnotatedIPFDensity()` enforces `config.imageWidth == config.imageHeight` with an early return and error message.

### Canvas size calculation

To avoid lossy scaling of the density image, `canvasDim` is computed so that `legendWidth` (the space available for the image after margins) equals `imageDim` exactly:
```
legendWidth = canvasDim - 2 * floor(canvasDim / 7)
```
Solving: `canvasDim = ceil(imageDim * 7 / 5)`

For imageDim=1024: canvasDim=1434, legendWidth=1434 - 2*204 = 1026 ≈ 1024. Close enough — canvas_ity handles the minor scaling. For pixel-perfect output, the density images could be generated at exactly `legendWidth` pixels, but the ~0.2% difference is imperceptible.

### Color order conventions

- `CreateIPFLegend()` and `createIPFColorImage()` both return pixels packed as uint32 via `RgbColor::dRgb()`: on little-endian systems the byte layout is `[B, G, R, A]`
- `ConvertColorOrder()` swaps bytes 0↔2: `[B,G,R,A] → [R,G,B,A]` (RGBA for canvas_ity)
- `MirrorImage()` flips rows vertically (image drawn with +Y down, canvas_ity uses +Y up)
- `annotateIPFImage()` handles both transforms internally, then removes alpha at the end
- Final output is **RGB** (3 components)

### The `drawIPFAnnotations` parameter signature

The `margins` parameter uses `const std::vector<float>&` (pass by const reference) for consistency. The existing `DrawFullCircleAnnotations` free functions use pass-by-value. When converting, change the parameter to `const std::vector<float>&` to match the new convention.

### Density always uses SST-only view

`generateAnnotatedIPFDensity()` always passes `generateEntirePlane = false` to `annotateIPFImage()`. This is intentional: IPF density plots display data within the Standard Stereographic Triangle, not the full stereographic circle.
