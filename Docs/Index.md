# Documentation for EbsdLib

EbsdLib is primarily used in the [DREAM3D](https://www.bluequartz.net) family of applications and libraries.

## Rotation Point Groups

The PDF is courtesy of Dr. Anthony Rollett from Carnegie Mellon University. The original
URL is [http://pajarito.materials.cmu.edu/lectures/L3-OD_symmetry-21Jan16-slide_50-operators.pdf](http://pajarito.materials.cmu.edu/lectures/L3-OD_symmetry-21Jan16-slide_50-operators.pdf)

## Hexagonal Cartesian Conventions: X‖a vs X‖a*

EbsdLib v3.0 aligned its hexagonal and trigonal direction conventions to `X‖a*`, matching
MTEX and Oxford Instruments / HKL acquisition systems. (EDAX/TSL/OIM Analysis use the
other convention, `X‖a`.) The 30° rotation between the two conventions is what caused
the original `(10-10)` and `(2-1-10)` pole-figure mismatches before the v3 changes.

**As of 3.1.0 the public default is `X‖a` (TSL/EDAX)**, matching legacy DREAM3D and the
rest of EbsdLib. The internal canonical SymOps tables remain `X‖a*` (MTEX-validated); only
the *default* selected by the IPF/pole-figure APIs and config structs changed. IPF colors
are convention-invariant — only pole-figure positions and the legend / pole-figure family
labels follow the basis.

![X parallel a-star convention](x_parallel_a_star_convention.svg)

Position-space validation across all 11 Laue classes lives in
[`Data/Pole_Figure_Validation/`](../Data/Pole_Figure_Validation/ReadMe.md).

---

# Release Notes — EbsdLib 3.1.1

EbsdLib 3.1.1 is the "misorientation analysis + correctness" release. It adds
tools for calculating and evaluating misorientation distribution functions
(MDFs), and fixes several long-standing Laue-class fundamental-zone and Schmid
factor defects inherited from the legacy OrientationLib implementation.

## New features

### Misorientation kernel density estimation

- `MisorientationKDE` accumulates weighted misorientations on the Laue-class MDF
  fundamental-zone grid and evaluates the resulting density at arbitrary
  misorientation quaternions.
- KDE evaluation is symmetrized over the crystal symmetry operators and grain
  exchange, and is normalized to a mean density of one over SO(3).
- `computeAngleCurve()` extracts a misorientation-angle distribution together
  with the corresponding random-reference distribution.
- Each occupied bin retains the weighted circular mean of its observations as
  its kernel center. This avoids the low-angle bias caused by snapping narrow
  kernels to geometric bin centers, particularly for hexagonal phases.

### SO(3) kernel and random-angle reference

- `SO3DeLaValleePoussinKernel` implements the de la Vallée Poussin orientation
  kernel, including half-width conversion, normalization, and finite support.
- `random_angle_distribution::Compute()` and `MaxMisorientationAngle()` provide
  analytical random misorientation-angle distributions and limits for the
  supported Laue classes.

## Correctness fixes

### Misorientation fundamental-zone folds

- Corrected `getMDFFZRod()` for cubic-low, trigonal, trigonal-low,
  hexagonal-low, tetragonal, and tetragonal-low symmetry. The corrected folds
  preserve the misorientation angle and use the proper rotational-symmetry
  sector for each Laue class.
- Implemented the previously stubbed triclinic, monoclinic, and orthorhombic
  folds, so MDF calculations for those phases no longer abort with
  `method_not_implemented`.
- Corrected equatorial boundary handling in the hexagonal, trigonal, and
  tetragonal folds.

### ODF and Schmid-factor calculations

- Fixed the tetragonal-low ODF dimension constant that caused
  `determineEulerAngles()` to return NaN Euler angles for all sampled bins.
- Replaced truncated cubic slip-system normalizers with the full-precision
  `sqrt(3)` and `sqrt(2)` constants, preventing Schmid factors from exceeding
  the physical maximum of 0.5 because of rounding bias.
- Initialized every output of the automatic `getSchmidFactorAndSS()` overloads
  on all code paths. Unsupported Laue classes now return defined zero values
  instead of stale or indeterminate angle components.

## Validation evidence

- Fundamental-zone tests cover all 11 Laue classes, including symmetry
  equivalence, angle preservation, idempotence, boundary behavior, and guards
  against over-folding.
- Kernel constants and random-angle distributions are checked against MTEX
  6.1.0 reference values.
- Cubic and hexagonal MDF angle curves are cross-checked against MTEX, and a
  correlated 60° about `<111>` (Σ3) distribution guards against regressions
  that flatten a textured MDF toward the random reference.
- Schmid-factor tests pre-poison output values to ensure the stub Laue classes
  explicitly reset every output.

## Maintenance

- Updated the formatting workflows to run for both pull requests and pushes,
  using clang-format 19.

---

# Release Notes — EbsdLib 3.1.0

EbsdLib 3.1 is the "discrete pole figures + TSL-by-default" release. It builds
on the 3.0 MTEX-correctness work with a new vector-marker pole-figure renderer
and a switch of the public default hexagonal/trigonal basis back to `X‖a`
(TSL/EDAX). The two themes:

1. **Discrete (vector-marker) pole figures.** A new renderer draws each pole as
   a crisp, decimated vector marker instead of rasterizing it as pixels —
   sharper figures, controllable marker size, and a hard cap on overdraw for
   million-pole datasets.
2. **`X‖a` (TSL) is the default convention again.** 3.0 shipped `X‖a*` (MTEX) as
   the public default; 3.1 returns the default to `X‖a` to match legacy DREAM3D
   and the rest of EbsdLib. The internal canonical tables are unchanged
   (`X‖a*`); only the default the public APIs/structs select changed.

Detailed API reference: [`v3_api_reference.md`](v3_api_reference.md).

## Breaking changes

### Default hexagonal/trigonal convention is now `X‖a` (TSL)

The default value of `HexConvention` flipped from `XParallelAStar` (3.0) to
`XParallelA` (3.1) in every public surface that selects it:

- `LaueOps::generateIPFTriangleLegend(...)` — the `conv` parameter, which was
  **required** in 3.0, now defaults to `ebsdlib::HexConvention::XParallelA`.
- `PoleFigureConfiguration_t::hexConvention` — now defaults to `XParallelA`.
- `CompositePoleFigureConfiguration_t::hexConvention` — now defaults to `XParallelA`.
- `InversePoleFigureConfiguration_t::hexConvention` — now defaults to `XParallelA`
  (affects only the SST label annotation; IPF colors are convention-invariant).

Effect: hexagonal/trigonal **pole-figure positions** and **legend/family labels**
render in the TSL basis by default. IPF *colors* are unaffected (convention-
invariant). Callers that want the 3.0 behavior must now pass
`HexConvention::XParallelAStar` explicitly (e.g. to compare side-by-side with
MTEX). Cubic / tetragonal / orthorhombic / monoclinic / triclinic are unaffected.

### Pole-figure family labels: brace notation, convention-independent

`getDefaultPoleFigureNames()` now returns **plane-family brace notation** —
`{0001}`, `{10-10}`, `{11-20}`, `{001}`, `{011}`, `{111}`, etc. — for every
Laue class, and the labels are **identical under both conventions**. The 3.0
behavior of switching the hexagonal a-family label (`<10-10>` ↔ `<2-1-10>`) with
the convention is removed; both conventions now report `{11-20}` for that family
(the family identity is convention-independent; only the on-figure dot positions
rotate by 30°). Downstream code that parsed the old angle-bracket strings, or
that relied on the per-convention label switch, must update.

### Rendered-figure differences

Pole figures now render **+Y-up** by default (`flipFinalImage`), and discrete
figures use the vector-marker renderer rather than per-pixel stamping. Output is
not byte-identical to 3.0. Any pixel-level exemplars pinned against 3.0 pole
figures must be regenerated.

## New features

### Discrete vector-marker pole figure renderer

- `DiscretePoleFigureCompositor` renders discrete pole figures by stamping
  opaque marker sprites, with a `MarkerOccupancyGrid` that decimates
  overlapping markers (bounded overdraw for >1M-pole inputs).
- `ebsdlib::GeneratePoleFigureComposite(config)` is the dispatch entry point:
  it routes `discrete && !discreteHeatMap` figures to the vector-marker
  renderer and everything else to the raster `PoleFigureCompositor`. Prefer
  this over calling a compositor directly.
- `CompositePoleFigureConfiguration_t` gained a `DiscreteMarkerStyle`
  (`markerStyle.radiusFraction`) to control marker size.
- `StereographicProjectUpperHemisphere(...)` projection helper.
- Shared title / axis / info-block chrome extracted into a reusable
  `PoleFigureChrome` module.

### IPF legends annotate their convention

Hexagonal and trigonal IPF triangle legends now print a small
`Convention: X||a (TSL)` (or `X||a* (MTEX/Oxford)`) sub-line under the title,
so a legend is self-documenting. Non-hex/trig classes are unchanged.

## Apps that changed

| App | Status |
| --- | ------ |
| `generate_ipf_legends` | defaults to `X‖a`; prints the absolute output directory at start/finish; regenerates the per-class legend + pole-figure matrix as PNG |
| `render_ebsd` | `--convention` now defaults to `x_a` (TSL) |

## Validation evidence

- **Strict IPF-color corners.** `LaueOpsTest::IPFColor_SSTCorners` asserts the
  standard-stereographic-triangle corners render as pure primaries for **every**
  Laue class: apex (`[0001]`/`[001]`) = red, η-min edge = green, η-max edge =
  blue (exact primaries for cubic and hex 6/mmm; dominant-channel just inside the
  edge for the wider-wedge classes, which sit on a fold boundary).
- **Discrete-render performance guard.** A hidden test renders a >1M-pole
  discrete figure to bound decimation cost.
- **MTEX positions still validated.** `PoleFigurePositionTest` continues to pass
  against the MTEX golden (`X‖a*`). `PoleFigureLaueComparisonTest` now routes
  through `GeneratePoleFigureComposite` (vector markers) and pins `X‖a*` locally
  to stay aligned with its MTEX comparison script.

---

# Release Notes — EbsdLib 3.0.0

EbsdLib 3.0 is the "MTEX-compatible pole figures and IPF coloring" release.
The two themes are:

1. **Crystallographic correctness for hexagonal / trigonal systems** —
   direction conventions, basal-plane plane families, and pole-figure
   positions now match MTEX out of the box (validated to `< 10⁻⁷` against
   MTEX 6.1.0 across all 11 Laue classes).
2. **Pluggable IPF coloring** — `ColorKeyKind` enum selects between TSL
   (DREAM3D-legacy), PUCM (perceptually uniform, Nolze 2016), and
   Nolze-Hielscher color keys at the call site, with optional grid-snapped
   ("flat-shaded") variants for MTEX-style legends.

Detailed API reference: [`v3_api_reference.md`](v3_api_reference.md).

## Breaking changes

Source consumers of EbsdLib must touch each of these only if they were
calling the affected symbol. The simplnx / DREAM3DNX / DREAM3D_Plugins
trees compile clean against v3 without source changes (audited; see the
v3 release checklist).

### Public LaueOps signature changes

- `generateSphereCoordsFromEulers(eulers, c1, c2, c3)` →
  `generateSphereCoordsFromEulers(eulers, c1, c2, c3, ebsdlib::HexConvention conv)`.
  Cubic / tetragonal / orthorhombic / monoclinic / triclinic overrides
  ignore `conv`; pass `HexConvention::NotApplicable`. Hex and trigonal
  overrides honor it.
- `getDefaultPoleFigureNames()` →
  `getDefaultPoleFigureNames(ebsdlib::HexConvention conv)`. Hex/trig classes
  return different labels per convention (`<10-10>` ↔ `<2-1-10>` shuffles);
  cubic/tet/ortho/mono/triclinic return the same labels regardless.
- `generateIPFTriangleLegend(int imageDim, bool generateEntirePlane)` →
  `generateIPFTriangleLegend(int imageDim, bool generateEntirePlane,
  ebsdlib::HexConvention conv, ebsdlib::ColorKeyKind kind =
  ebsdlib::ColorKeyKind::TSL, bool gridded = false)`. The `kind` and
  `gridded` arguments default; `conv` does not, because the legend itself
  draws different labels under the two bases.
- `generateIPFColor(eulers, refDir, convertDegrees)` →
  `generateIPFColor(eulers, refDir, convertDegrees, ebsdlib::ColorKeyKind kind =
  ebsdlib::ColorKeyKind::TSL)`. The `kind` defaults to TSL — pre-v3 callers
  recompile unchanged and get the same colors. Note that IPF coloring is
  now **convention-invariant** (it operates on the sample-frame reference
  direction, which never sees the basal basis); the pre-v3 `HexConvention`
  parameter some patches briefly added was removed before release.

### Enum shifts

  Code that uses the named values is unaffected. Code that
  `static_cast<HexConvention>(int)` from a UI index needs the cast target
  to match the new ordering — confirm by reading the new enum, not by
  remembering pre-v3 integer values.

### Symmetry orbit / direction-table changes

Several low-symmetry Laue classes had their symmetry orbits expanded as
part of the SymOps refactor (PR 2a–2d). Pole figures rendered before/after
v3 will not be byte-identical for these classes — the sym-op count for the
same Laue class changed. Position-space tests pass (`< 10⁻⁷` vs MTEX), so
this is a correctness fix, not a regression. If you pin byte-level pixel
exemplars at the simplnx layer (the way the now-deprecated
`PoleFigure_Exemplars_v5.tar.gz` did), regenerate those baselines against
v3 output.

### Image output format

- `make_pole_figure`, `make_ipf`, `render_ebsd`, and the simplnx
  `WritePoleFigureFilter` / `WriteIPFImageFilter` filters now emit PNG
  (via STB image) instead of TIFF. Consumers that watched for `*.tiff`
  pole-figure output need to watch for `*.png` instead. The pixel content
  is the same up to PNG-encoder differences from libtiff.

## New features

### `ColorKeyKind` and per-class color keys

```cpp
enum class ColorKeyKind : uint8_t {
  TSL = 0,            // EDAX / DREAM3D-legacy IPF coloring
  PUCM = 1,           // Perceptually uniform (Nolze 2016)
  NolzeHielscher = 2  // Nolze-Hielscher color key
};
```

Each `LaueOps` subclass owns a per-class singleton for each kind. The
TSL singleton is shared across all classes (it's the standard EDAX
mapping); PUCM is parameterized by rotation point group; Nolze-Hielscher
is parameterized by the fundamental sector. Callers select among them at
the call site by passing the kind enum to `generateIPFColor` or
`generateIPFTriangleLegend` — instances stay stateless, no setter API.

### `HexConvention::NotApplicable` sentinel

Cubic / tetragonal / orthorhombic / monoclinic / triclinic Laue classes
have no `X‖a` vs `X‖a*` distinction to make. The `NotApplicable` value
is the right thing for those classes to pass when one of the new
hex/trig-aware APIs requires the convention parameter. The hex/trig
overrides assert if they receive `NotApplicable`, and the non-hex/trig
overrides ignore the parameter regardless of its value.

### Gridded color-key legends

`generateIPFTriangleLegend(..., gridded=true)` wraps the selected color
key in a `GriddedColorKey` (~1° eta × chi resolution), producing
MTEX-style flat-shaded cells instead of a smooth continuous gradient.
Useful for visually matching MTEX legend renders side-by-side.

### `render_ebsd` CLI driver

New app at `Source/Apps/render_ebsd.cpp`. Single-binary entry point for
pole figure, IPF map, and IPF legend rendering against a `.ang` / `.ctf`
input — useful for CI smoke tests and reference renders. Rejects missing
output directories and bad positional arguments before doing any work.

### `InversePoleFigureConfiguration_t` carries `HexConvention`

The IPF rendering config struct now has a `hexConvention` field so the
hex/trig-aware code paths (legend labels, basal-plane direction tables)
can be routed through the same configuration object the existing
`PoleFigureConfiguration_t` uses.

### Cropped IPF triangle legends

`generateIPFTriangleLegend` output is now cropped to the SST contents
(plus a small margin). No more wasted whitespace on classes whose
fundamental sector occupies only a fraction of the unit triangle.

## Apps that changed

| App | Status |
| --- | ------ |
| `make_pole_figure` | rewritten — STB/PNG output, MTEX-compatible pole positions, HexConvention-aware family labels |
| `make_ipf`          | rewritten — STB/PNG output, ColorKeyKind dispatch |
| `generate_ipf_legends` | new flags for color key + gridded variant; emits per-class legend matrix |
| `render_ebsd`       | **new** — single-binary pole-figure / IPF map / IPF legend renderer for `.ang` / `.ctf` inputs |
| `generate_pole_figure` | unchanged surface, but inherits the renderer overhaul |
| `generate_ipf_from_file` | unchanged surface |


## Validation evidence

- **Position-space test.** `PoleFigurePositionTest` (Catch2) compares
  EbsdLib pole figure positions against an MTEX-generated golden CSV
  across 396 buckets (12 canonical orientations × 11 Laue classes ×
  3 plane families). Worst max-distance: `6.29 × 10⁻⁸` at `1e-5`
  tolerance. See [`Data/Pole_Figure_Validation/ReadMe.md`](../Data/Pole_Figure_Validation/ReadMe.md).
- **Renderer reproducibility.** `PoleFigureCompositorTest::All_Laue_Classes`
  pins byte-level renderer output across every Laue class.
- **HexConvention plumbing.** simplnx `WritePoleFigureFilter` has two
  test cases: a mask-effectiveness test (`Pole_Figure_Exemplars_v6`) and
  a HexConvention plumbing test that asserts both the intensity array
  AND the composite RGB image differ when switching X‖a → X‖a* on hex
  data.
- **Convention story.** The geometric picture is in
  [`x_parallel_a_star_convention.svg`](x_parallel_a_star_convention.svg);
  the canonical internal direction tables are `X‖a*`, matching MTEX.
