# Various Bits of Documentation for EbsdLib

EbsdLib is primarily used in the [DREAM3D](https://www.dream3d.io) family of applications and libraries.

## Rotation Point Groups

The PDF is courtesy of Dr. Anthony Rollett from Carnegie Mellon University. The original
URL is [http://pajarito.materials.cmu.edu/lectures/L3-OD_symmetry-21Jan16-slide_50-operators.pdf](http://pajarito.materials.cmu.edu/lectures/L3-OD_symmetry-21Jan16-slide_50-operators.pdf)

## Hexagonal Cartesian Conventions: X‖a vs X‖a*

EbsdLib v3 aligned its hexagonal and trigonal direction conventions to `X‖a*`, matching
MTEX and Oxford Instruments / HKL acquisition systems. (EDAX/TSL/OIM Analysis use the
other convention, `X‖a`.) The 30° rotation between the two conventions is what caused
the original `(10-10)` and `(2-1-10)` pole-figure mismatches before the v3 changes.

![X parallel a-star convention](x_parallel_a_star_convention.svg)

Position-space validation across all 11 Laue classes lives in
[`Data/Pole_Figure_Validation/`](../Data/Pole_Figure_Validation/ReadMe.md).

The full design rationale and the v2→v3 ordering finding (why the canonical
internal direction tables remain in `X‖a*` rather than the legacy `X‖a`)
is in [`Code_Review/v3_phase0_design_notes.md`](../Code_Review/v3_phase0_design_notes.md)
§16.

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

### Removed API

- `LaueOps::setColorKey()` / `getColorKey()` / `m_ColorKey` member.
  Coloring scheme is selected at the call site by passing a `ColorKeyKind`,
  not by mutating long-lived state on the LaueOps object. Per-class
  singletons for each kind live in file-local `keyForKind()` helpers; the
  base class doesn't see them. This makes LaueOps instances stateless
  again and removes a thread-safety footgun.
- `LaueOps::setLegendRenderMode()` + the `LegendRenderMode` enum. The
  gridded / interpolated choice now travels as a `bool gridded` argument
  on `generateIPFTriangleLegend`, not as object-level state. No external
  call sites were found in the consumer audit.
- `generateRodriguesColor(r1, r2, r3, HexConvention)` is now
  `generateRodriguesColor(r1, r2, r3)`. Rodrigues-space coloring is
  convention-invariant for the same reason IPF coloring is.

### Enum shifts

- `HexConvention` value order changed during stabilization to align with
  the DREAM3DNX UI dropdown index order:

  ```
  enum class HexConvention : uint8_t {
    XParallelA = 0,        // was 1
    XParallelAStar = 1,    // was 2
    NotApplicable = 2      // was 0
  };
  ```

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

## Bug fixes

- **PUCM `cubicToHemi` / `cubicLowToHemi` thread race** (commit `6084a50`).
  The wlenthe lookup-table init was non-atomic; two threads entering
  PUCM coloring on different LaueOps instances could interleave init and
  corrupt the table. Init is now done under `std::call_once`.
- **`PoleFigureCompositor` dropped `config.hexConvention`** (PR 2g,
  commit `9395592`). The compositor was passing a freshly-default-constructed
  `HexConvention` to the per-family renderers instead of propagating the
  one the caller asked for. Fixed; covered by the simplnx
  `WritePoleFigureFilter: HexConvention choice reaches algorithm` plumbing
  test.
- **Stray vertical column in 622 IPF legend** (commit `873e61c`).
- **`GriddedColorKey::direction2Color` ignored `angleLimits`** (commit
  `2c20533`).
- **`GriddedColorKey` cell-center snap pushing `(eta, chi)` outside the
  SST** (commit `89aca99`).
- **`GriddedColorKey` eta clamp** (commit `b3aafc4`). Eta is a periodic
  azimuth and must not be clamped; only chi (the polar angle) should be
  clamped to its SST limit.

## Migration recipes (for external / out-of-tree callers)

The in-tree consumer audit (Phase 3 of the release checklist) found zero
call sites for the patterns below. They're listed here for forks /
third-party callers that may not be in the audit set.

### Pattern A — replace state mutation with call-site dispatch

**Before:**
```cpp
op->setColorKey(std::make_shared<PUCMColorKey>(op->getRotationPointGroup()));
auto rgb = op->generateIPFColor(eulers, refDir, false);
```

**After:**
```cpp
auto rgb = op->generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::PUCM);
```

The PUCM singleton is owned and lazy-initialized inside the LaueOps
subclass; you don't construct it. PUCM init is now thread-safe.

### Pattern B — gridded legend rendering

**Before:**
```cpp
op->setLegendRenderMode(LegendRenderMode::GridInterpolated, 1.0);
auto img = op->generateIPFTriangleLegend(N, fullPlane);
```

**After:**
```cpp
auto img = op->generateIPFTriangleLegend(
    N, fullPlane, ebsdlib::HexConvention::XParallelAStar,
    ebsdlib::ColorKeyKind::TSL, /*gridded=*/true);
```

### Pattern C — drop the `HexConvention` from IPF / Rodrigues color calls

**Before:**
```cpp
auto rgb = op->generateIPFColor(eulers, refDir, false, HexConvention::XParallelA);
auto rod = op->generateRodriguesColor(r1, r2, r3, HexConvention::XParallelA);
```

**After:**
```cpp
auto rgb = op->generateIPFColor(eulers, refDir, false);
auto rod = op->generateRodriguesColor(r1, r2, r3);
```

IPF and Rodrigues coloring are convention-invariant: they operate on the
sample-frame reference direction (IPF) or on a Rodrigues vector in
crystal space (Rodrigues), neither of which sees the basal basis. Pre-v3
overloads that accepted `HexConvention` here were silently dropping it on
the floor; v3 removes the dead parameter.

## Apps that changed

| App | Status |
| --- | ------ |
| `make_pole_figure` | rewritten — STB/PNG output, MTEX-compatible pole positions, HexConvention-aware family labels |
| `make_ipf`          | rewritten — STB/PNG output, ColorKeyKind dispatch |
| `generate_ipf_legends` | new flags for color key + gridded variant; emits per-class legend matrix |
| `render_ebsd`       | **new** — single-binary pole-figure / IPF map / IPF legend renderer for `.ang` / `.ctf` inputs |
| `generate_pole_figure` | unchanged surface, but inherits the renderer overhaul |
| `generate_ipf_from_file` | unchanged surface |

## simplnx UI integration

Downstream DREAM3DNX users will see new dropdown parameters on the
following simplnx filters once both sides are on v3:

- `WritePoleFigureFilter` — `hex_convention_index` (X‖a / X‖a*)
- `ComputeIPFColorsFilter` — `color_key_index` (TSL / PUCM / Nolze-Hielscher)
- `ComputeFaceIPFColoringFilter` — `color_key_index` (same set)

The simplnx `Convert Hex/Trig Euler Angles Between Cartesian Conventions`
filter handles the data-side basis rotation (e.g. when feeding `.ang`
X‖a data into a downstream pipeline that expects X‖a* internally).

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
- **Convention story.** See `Code_Review/v3_phase0_design_notes.md` §16
  for the canonical-source-of-truth decision and the geometric picture
  in [`x_parallel_a_star_convention.svg`](x_parallel_a_star_convention.svg).
