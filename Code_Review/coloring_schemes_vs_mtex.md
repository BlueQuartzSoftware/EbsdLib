# EbsdLib IPF Coloring Schemes vs. MTEX

A working document tracking how EbsdLib's IPF (Inverse Pole Figure) color
keys are defined and how they match or diverge from MTEX's equivalents.
Add to this as new classes / keys / convention questions come up.

Audience: maintainers reviewing why two visually similar IPF triangles do
not produce identical pixels. Goal: nobody has to re-derive the same math
twice.

---

## TSL color key

### The conceptual goal

Both EbsdLib and MTEX expose a "TSL" color key whose intent is the
familiar EBSD vendor color scheme: pure red at the [0001]/[001]
fundamental-zone corner, green and blue at the other two corners of the
standard stereographic triangle, with a soft white-ish highlight near the
interior. The name "TSL" comes from EDAX/TSL OIM Analysis software, which
popularized the convention.

The two implementations agree at the fundamental-zone corners (the corner
test in `IPFLegendTest::CAxisIsRed` passes for every Laue class), but
they use materially different math to color the interior of the
triangle. They are NOT pixel-equivalent — even for cubic m-3m where they
look the closest, individual pixel values can differ by a few percent.

### EbsdLib's TSL formula

File: `Source/EbsdLib/Utilities/TSLColorKey.cpp`

The 3-arg overload (called by `LaueOps::computeIPFColor` per pixel):

```cpp
Vec3 TSLColorKey::direction2Color(double eta, double chi,
                                  const Vec3& angleLimits) const {
  double etaMin = angleLimits[0];
  double etaMax = angleLimits[1];
  double chiMax = angleLimits[2];

  double r = 1.0 - chi / chiMax;
  double b = std::abs(eta - etaMin) / (etaMax - etaMin);
  double g = 1.0 - b;
  g *= chi / chiMax;
  b *= chi / chiMax;

  r = std::sqrt(r);
  g = std::sqrt(g);
  b = std::sqrt(b);

  double maxVal = std::max({r, g, b});
  if(maxVal > 0.0) { r /= maxVal; g /= maxVal; b /= maxVal; }

  return {clamp01(r), clamp01(g), clamp01(b)};
}
```

Inputs are `(eta, chi)` — the polar coordinates of the FZ-folded crystal
direction in the standard stereographic triangle — plus the angle limits
of that triangle for the active Laue class.

Behavior at the corners:
- `chi = 0`           → c-axis vertex → `r = 1, g = 0, b = 0`           → red.
- `chi = chiMax, eta = etaMin` → green vertex → `r = 0, b = 0, g → 1`   → green after normalize.
- `chi = chiMax, eta = etaMax` → blue vertex  → `r = 0, g = 0, b → 1`   → blue after normalize.

Interior behavior:
- The `sqrt` of each channel is a perceptual brightness lift.
- The `max-normalize` step pushes the brightest channel up to 1 every
  pixel, so colors saturate quickly. This is what gives the
  "white-ish" highlight near the triangle's incenter — the spot where
  all three of `r`, `g`, `b` happen to be similar.

The 0-arg overload `direction2Color(const Vec3& direction)` falls back
to a hardcoded `m_DefaultAngleLimits = {0, π/4, acos(1/√3)}` (cubic m-3m
limits). Important for the gridded code path — see "Gridded rendering"
below.

Reference for the corner color test: `IPFLegendTest::CAxisIsRed` at
`Source/Test/IPFLegendTest.cpp`.

### MTEX's TSL formula

File: `mtex-6.1.0/plotting/directionColorKeys/TSLDirectionKey.m`

```matlab
function rgb = direction2color(dM, h, varargin)
  h = h.project2FundamentalRegion(dM.sym);
  center = dM.sR.center;
  v = dM.sR.vertices;
  if ~isempty(v), v = v(1); else, v = dM.sym.aAxisRec; end

  [radius, rho] = polarCoordinates(dM.sR, h, center, v);

  radius = 0.5 + radius./2;     % "white center" (but see below)
  v = vector3d('rho', rho, 'theta', radius.*pi);
  v = dM.colorPostRotation * v;

  rgb = ar2rgb(mod(v.rho./2./pi, 1), v.theta./pi, ...
               get_option(varargin,'grayValue',1), 'noHueCorrection');
end
```

This is a polar / HSL scheme — fundamentally different from EbsdLib's
chi-eta formula:

1. `polarCoordinates(sR, v, center, ref)` returns `(radius, rho)` where
   `radius` is normalized distance from the FZ centerpoint to the FZ
   boundary along the line through `v`, and `rho` is the azimuthal angle
   of `v` around the centerpoint.
2. `radius = 0.5 + radius./2` remaps `[0, 1]` to `[0.5, 1.0]`.
3. `ar2rgb` interprets the polar coordinates as HSL: hue from `rho`,
   lightness from the remapped radius (with default `grayValue=1` this
   simplifies to `L = radius`, `S = 1`).
4. Convert HSL → HSV → RGB.

Behavior at the corners and boundary:
- FZ center → `radius=0` → remapped `L=0.5` → fully saturated color of
  some specific hue (whichever hue corresponds to the chosen `ref`
  direction).
- FZ boundary → `radius=1` → remapped `L=1` → **white** in HSL, so
  white pixels along the FZ perimeter.
- The hue (`rho`) cycles a full 360° around the FZ — which is why MTEX
  legends can show a complete rainbow around the boundary.

Note: the inline comment `"% white center"` in `TSLDirectionKey.m` is
misleading — the math actually produces white at the BOUNDARY and
saturated color in the center. This matches the rendered MTEX legends.

`ar2rgb.m` also applies an optional hue-correction step that biases the
hue distribution to give roughly equal area to red, green, and blue
regions of the wheel; EbsdLib's TSLColorKey has no equivalent.

### Why they don't agree pixel-for-pixel

The two schemes share the *same vertex colors* (red / green / blue at
the three FZ corners) but use **different interior interpolation**:

| Property                     | EbsdLib TSL                                   | MTEX TSL                                        |
| ---------------------------- | --------------------------------------------- | ----------------------------------------------- |
| Inputs                       | `(eta, chi)`                                  | `(rho, radius)` — polar in the FZ               |
| Brightness control           | `sqrt(r), sqrt(g), sqrt(b)` then max-normalize| HSL `L = 0.5 + radius/2`                        |
| Hue around boundary          | Three corner-anchored linear ramps            | Smooth full hue cycle (HSV)                     |
| Center of FZ                 | Pure red (chi=0)                              | Saturated color of one specific hue (not red)   |
| Triangle interior            | Brightest near the triangle incenter          | Brightest along the FZ boundary                 |
| Hue correction               | None                                          | Optional Gaussian-bumpy hue mapping             |

Practically, the two visualizations agree on coarse interpretations
("orange means roughly between [001] and [011]") but disagree at the
pixel level. For visual side-by-side comparison the schemes look
broadly similar for cubic m-3m, hex 6/mmm, etc., where the FZ is small
and the chi-eta formula approximates a polar map. They look very
different for Triclinic where the FZ is the entire upper hemisphere and
the chi-eta formula has more space to disagree with a polar/HSL scheme.

---

## Per-Laue-class status

Status legend:
- ✅ matches MTEX visually (small numerical differences only)
- ⚠️  recognizable as the same scheme but visibly different
- ❌ fundamentally different appearance

| Rotation PG | Class             | EbsdLib TSL ↔ MTEX TSL | Notes |
| ----------- | ----------------- | ---------------------- | ----- |
| 432         | Cubic m-3m        | ✅                     | Standard cubic IPF triangle. Both schemes give R/G/B at corners; interior gradients differ slightly but the visual impression matches. |
| 23          | Cubic m-3         | ✅                     | Same triangle as 432 in EbsdLib (same direction triplets after the 4th-{011}-direction typo fix in CubicLowOps). |
| 622         | Hexagonal 6/mmm   | ✅                     | After the X\|\|a\* convention fix, vertex labels match. Interior coloring close. |
| 6           | Hexagonal 6/m     | ⚠️                     | EbsdLib enumerates 6-fold orbit; vertex labels (`<10-10>`/`<11-20>`) match MTEX. Interior coloring close. |
| 422         | Tetragonal 4/mmm  | ✅                     | |
| 4           | Tetragonal 4/m    | ⚠️                     | After renaming third PF from `<010>` to `<110>` and enumerating the 4-fold orbit. |
| 32          | Trigonal -3m      | ⚠️                     | |
| 3           | Trigonal -3       | ⚠️                     | The two prismatic PFs share the same 3-fold orbit under -3 — both PFs render identically. |
| 222         | Orthorhombic mmm  | ✅                     | |
| 2           | Monoclinic 2/m    | ✅                     | |
| 1           | Triclinic -1      | ❌                     | Fundamentally different (see below). |

### Triclinic divergence

Triclinic is the loudest disagreement and worth its own subsection.

**MTEX FZ choice.** `fundamentalSector.m`, case 2 sets `N = +Z`, so the
fundamental region for `-1` is the **upper hemisphere half-disk**. The
inversion-equivalent of any direction in the lower hemisphere is its
upper-hemisphere antipode, so MTEX folds and renders only the upper half.

**EbsdLib FZ choice.** EbsdLib's Triclinic SST limits are
`(etaMin=0, etaMax=π, chiMax=π/2)`. The chi-eta formula uses
`|eta - etaMin| = |eta|`, which makes the result symmetric across the
y=0 line of the stereographic disk. Combined with the legend renderer
filtering only `|x|² + |y|² ≤ 1` for triclinic (`generateEntirePlane=true`),
the **full disk** is rendered with mirrored coloring above and below
y=0.

**Visible result.** MTEX shows a half-disk with a smooth HSV cycle
around its perimeter, white near the boundary, fully-saturated single
hue at the center. EbsdLib shows a full disk with red at the center,
greens/blues spreading outward, and a top/bottom mirror symmetry across
y=0.

These are not equivalent renderings of the same scheme. They are
different schemes applied to different fundamental regions. Matching
MTEX's Triclinic from EbsdLib would require:
1. Restricting the legend renderer to the upper hemisphere half-disk
   (or accepting both renders as legitimate).
2. Adding a polar-coordinate IPF color key (analogous to MTEX's
   `TSLDirectionKey`) so the interior coloring uses HSL/HSV instead of
   the chi-eta formula.

---

## Gridded rendering

EbsdLib's `GriddedColorKey` is a decorator that wraps any `IColorKey`
and provides MTEX-style flat-shaded 1° grid cells. After the bug fixes
in commits `2c20533`, `89aca99`, and `88ad1f9`, the gridded TSL output
matches the per-pixel TSL output across all 11 unique Laue classes:

- The 3-arg overload now honors the caller's `angleLimits` (was using
  cubic-default limits for every class).
- `eta` is passed through unchanged (was being wrapped to `[0, 2π]`,
  which broke negative-`etaMin` Laue classes Trig-3 and Trig-3m).
- Snapped `chi` is clamped to `[0, angleLimits[2]]` (cubic m-3m has
  variable `chiMax(eta)`; the snap could push `chi` past the chiMax
  passed in, producing NaN red along the curved boundary).
- Snapped `eta` is NOT clamped (clamping it broke Triclinic — the lower
  hemisphere of the legend disk relies on negative eta passing through
  to the `|eta|` symmetry of the chi-eta formula).

Regression coverage: `GriddedColorKey::HonorsAngleLimitsIn3ArgOverload`,
`GriddedColorKey::HandlesNegativeEta`,
`GriddedColorKey::BoundarySnapDoesNotProduceNaN`,
`GriddedColorKey::TriclinicNegativeEtaProducesColor`.

The GriddedColorKey output is therefore a faithful 1° flat-shading of
EbsdLib's TSL formula, but it does **not** match MTEX's TSL output any
better than the per-pixel version does — because the underlying color
math is still the EbsdLib chi-eta formula, not MTEX's polar/HSL formula.

---

## Validation against EDAX TSL reference (`EDAX_TSL_IPF.bmp`)

Test data and reference output supplied by EDAX:
- Input: `Data/ipf_color_tests/AllLaueClasses_RandO.ang` — 96×100 grid,
  one orientation per row, 12 phases (one per Laue class) repeated
  across 8-pixel-wide vertical strips.
- TSL reference: `Data/ipf_color_tests/EDAX_TSL_IPF.bmp` (533×511,
  ~5.33× upscaled from the .ang grid).
- PUCM reference (separate scheme, future work):
  `Data/ipf_color_tests/EDAX_PUCM_IPF.bmp`.

Comparison procedure:

```bash
make_ipf Data/ipf_color_tests/AllLaueClasses_RandO.ang /tmp/ebsdlib.png
# nearest-downsample EDAX_TSL_IPF.bmp to 96x100 with PIL/etc.
# diff against /tmp/ebsdlib.png pixelwise
```

Per-Laue-class result (mean per-channel diff out of 255, full strip):

| Phase | EDAX_TSL ↔ EbsdLib | EDAX_TSL ↔ MTEX |
| ----- | ------------------ | ---------------- |
| dihex 6/mmm        | **0.42** | 18.28 |
| triclinic          | 0.57     | 91.32 |
| cubic m-3m         | 0.64     | 14.05 |
| hex 6/m            | 0.91     | 11.02 |
| ditet 4/mmm        | 1.02     | 14.42 |
| mono b             | 1.02     | 39.60 |
| ortho mmm          | 1.28     | 13.86 |
| tetrahedral m-3    | 1.29     | 20.03 |
| tet 4/m            | 1.49     | 14.51 |
| trig -3            | 1.81     | 21.50 |
| ditrig -3m         | 2.65     | 71.76 |
| **mono c**         | **31.82** | 35.71 |
| **whole image**    | **3.74**, only 5.8 % of pixels differ at all | 30.51 |

Headline: **EbsdLib's TSL key matches EDAX's TSL output to within
sub-pixel accuracy on 11 of 12 Laue classes**. The remaining ~2%
"mismatch floor" on those 11 phases is consistent with antialiasing /
quantization from the 5.33× non-integer downsample of the EDAX BMP and
is not a real coloring difference. MTEX disagrees with EDAX as
documented above (polar-HSL vs chi-eta TSL formulas).

Mono-c is the one phase where EbsdLib disagrees with EDAX in a way
that's clearly not just downsample noise — added to "Open questions"
below.

---

## PUCM color key (perceptually uniform, EDAX-style)

EbsdLib now ships a third IPF color key, `PUCMColorKey`, ported from
William Lenthe's BSD-3 reference implementation
(`wlenthe/crystallography/orientation_coloring.hpp`, vendored at
`Source/EbsdLib/Utilities/wlenthe_orientation_coloring.hpp`).
PUCMColorKey is a thin dispatch wrapper that selects the correct
wlenthe entry-point per Laue class.

The implementation follows:
- Nolze, G. and Hielscher, R. *"Orientations Perfectly Colors."*
  J. Appl. Crystallogr. 49.5 (2016): 1786–1802.
- EDAX OIM Analysis "perceptually uniform" IPF color scheme (PUCM):
  see <https://www.edax.com/news-events/edax-blog/edax-blog-posts/improved-ipf-color-palettes>.

`make_ipf` accepts a third optional argument `tsl|pucm` to pick the
color key. PUCM constructs a per-rotation-point-group color key on
each LaueOps before rendering.

PUCM is also wired into the legend pipeline: every Laue class now
emits four EbsdLib legend variants per `IPFLegendTest` run:

```
<rpg>/tsl_ebsdlib_ipf_legend.png
<rpg>/tsl_gridded_ebsdlib_ipf_legend.png
<rpg>/nh_ebsdlib_ipf_legend.png
<rpg>/nh_gridded_ebsdlib_ipf_legend.png
<rpg>/pucm_ebsdlib_ipf_legend.png         <-- new
<rpg>/pucm_gridded_ebsdlib_ipf_legend.png <-- new
```

The PUCM legends use the same `LaueOps::generateIPFTriangleLegend`
machinery as TSL/NH; the rendering loop calls `generateIPFColor` per
pixel, which routes through the active `m_ColorKey`. Setting the key
to a `PUCMColorKey(rpg)` instance for the corresponding Laue class
just before rendering produces the perceptually uniform legend
without any other code path changes.

### Validation against EDAX PUCM reference (`EDAX_PUCM_IPF.bmp`)

Same input as the TSL validation (AllLaueClasses_RandO.ang, 96×100
grid, 12 phases). EDAX renders this with their PUCM color scheme and
ships it alongside the test data.

| Phase | EDAX_PUCM ↔ EbsdLib_PUCM mean diff |
| ----- | ----------------- |
| dihex 6/mmm     | **0.47** |
| triclinic       | 0.47     |
| cubic m-3m      | 0.87     |
| ortho mmm       | 1.19     |
| trig -3         | 1.25     |
| ditrig -3m      | 1.30     |
| mono b          | 1.79     |
| hex 6/m         | 1.91     |
| tetrahedral m-3 | 2.42     |
| tet 4/m         | 10.05    |
| ditet 4/mmm     | 11.78    |
| **mono c**      | **74.67** |
| **whole image** | **9.01**, 26.5% of pixels differ at all |

9 of 12 Laue classes match EDAX PUCM to within sub-pixel accuracy
(mean diff < 3 / 255). Mono-c is the same outlier we saw with the TSL
comparison and is therefore not a PUCM-specific issue — see Open
Questions.

`tet 4/m` and `ditet 4/mmm` have moderate divergences (mean ≈ 10–12).
Plausible causes: small fundamental-sector convention differences
between wlenthe's reference and what EDAX ships, or a subtle dispatch
issue in `PUCMColorKey` for those specific Laue classes. Worth
investigating; not yet diagnosed.

---

## Open questions

(Add as they come up.)

- **Monoclinic c-setting (`mono c`, point group `112/m`) — EbsdLib vs
  EDAX disagree** by mean=31.82 / max=244 (TSL) and mean=74.67 / max=255
  (PUCM) over ~46–100% of pixels in the AllLaueClasses_RandO comparison.
  Every other Laue class matches EDAX to within ~3 mean diff for both
  color keys. The fact that mono-c diverges in BOTH TSL and PUCM
  comparisons rules out a color-formula bug — strongly indicates a
  b-setting vs c-setting axis convention mismatch in EbsdLib's
  MonoclinicOps phase mapping or Euler-angle interpretation.
- **Tetragonal-low (4/m) and tetragonal-high (4/mmm) PUCM** have moderate
  divergence vs EDAX_PUCM (mean ≈ 10–12) while the same classes match
  exactly under TSL (mean ≈ 1–2). Suggests the wlenthe dispatch or the
  fundamental-sector convention is subtly different for these two
  classes specifically. The other 9 Laue classes are clean.

- The visual hex `<10-10>` and `<11-20>` PF positions match MTEX after
  the X\|\|a\* convention fix, but the *interior* coloring of the
  prismatic-orbit pole figures might still differ by a constant 30°
  rotation in some classes. Worth a closer look.
- For Triclinic: do we want EbsdLib to also restrict to the upper
  hemisphere (matches MTEX, breaks back-compat with old EbsdLib
  outputs), or keep the full-disk render and document it as a
  deliberate divergence?
- The `NolzeHielscherColorKey` should be the right comparand for MTEX's
  `ipfHSVKey`. We have output for both; the side-by-side comparison
  for non-cubic classes should be revisited now that the GriddedColorKey
  bugs are fixed.

---

## How to regenerate the comparison

```bash
cd /Users/mjackson/Workspace7/DREAM3D-Build/ebsdlib-Release
Bin/EbsdLibUnitTest "ebsdlib::IPFLegendTest::MTEXCompare_AllLaueClasses"
# Then in MATLAB:
#   run('Code_Review/compare_ipf_legends_all_laue.m')
```

Output lands in `<build>/Testing/Temporary/IPFComparison/<rpg>/`:
- `tsl_ebsdlib_ipf_legend.png` — per-pixel TSL
- `tsl_gridded_ebsdlib_ipf_legend.png` — 1° gridded TSL
- `nh_ebsdlib_ipf_legend.png` — per-pixel Nolze-Hielscher
- `nh_gridded_ebsdlib_ipf_legend.png` — 1° gridded NH
- `tsl_mtex_ipf_legend.png` — MTEX `ipfTSLKey`
- `nh_mtex_ipf_legend.png` — MTEX `ipfHSVKey`

The TSL pair is *intended* to match per-class within "broadly similar"
visual quality, with the documented Triclinic exception. The NH/HSV
pair is the more rigorous match (both use polar HSV-cycle schemes).
